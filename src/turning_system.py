# Turning System - Physics-based circular arc turning
# Separate from Car class for clean architecture

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Tuple
from . import config


@dataclass
class TurnPlan:
    """Immutable geometry for a circular arc turn between two road segments."""
    
    # Arc geometry
    center_x: float
    center_y: float
    radius: float
    start_angle: float  # radians
    end_angle: float    # radians
    clockwise: bool
    
    # Road segments
    from_seg_idx: int
    to_seg_idx: int
    junction_node: str
    
    # Arc properties
    arc_length: float  # meters
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    
    # Progress tracking
    progress: float = 0.0  # 0.0 to 1.0 along arc
    
    # Calibrated correction (degrees) so heading is guaranteed to be
    # continuous with the car's actual heading at progress=0. The
    # tangent-heading formula below is derived from a circle
    # parametrization (cos/sin) that is rotated 90 deg relative to the
    # sin/-cos convention used to place the arc's center from the car's
    # heading, which otherwise produces a systematic ~90 deg discontinuity
    # right when the turn starts. See TurningSystem.plan_turn().
    heading_offset: float = 0.0  # degrees
    
    def get_point_on_arc(self, progress: float) -> Tuple[float, float, float]:
        """Get (x, y, heading) at given progress along arc.
        
        Args:
            progress: 0.0 to 1.0 along the arc
        
        Returns:
            (x, y, heading_degrees)
        """
        # Interpolate angle
        angle_diff = self.end_angle - self.start_angle
        
        # Handle wrapping
        if self.clockwise and angle_diff > 0:
            angle_diff -= 2 * math.pi
        elif not self.clockwise and angle_diff < 0:
            angle_diff += 2 * math.pi
        
        current_angle = self.start_angle + progress * angle_diff
        
        # Position on circle
        pppm = config.PIXELS_PER_METER
        x = self.center_x + self.radius * pppm * math.cos(current_angle)
        y = self.center_y + self.radius * pppm * math.sin(current_angle)
        
        # Heading (tangent to circle), calibrated to match the car's real
        # heading at progress=0 (see heading_offset docstring above)
        if self.clockwise:
            heading = math.degrees(current_angle - math.pi / 2)
        else:
            heading = math.degrees(current_angle + math.pi / 2)
        heading += self.heading_offset
        
        return x, y, heading % 360


class TurningSystem:
    """Calculates and manages physics-based circular arc turns."""
    
    # Mechanical minimum turning radius (steering-geometry limit): a real
    # car cannot turn tighter than this REGARDLESS of speed. Only the
    # centripetal-force limit (v²/a) scales with speed; at low speed the
    # mechanical limit dominates instead. Matches PhysicsValidator's
    # MIN_REALISTIC_RADIUS_M expectations (kept comfortably above it).
    MIN_MECHANICAL_RADIUS_M = 5.0
    
    def __init__(self, max_lateral_accel: float = 5.0):
        """Initialize turning system.
        
        Args:
            max_lateral_accel: Maximum lateral acceleration in m/s² (design constraint)
        """
        self.max_lateral_accel = max_lateral_accel
    
    def calculate_turning_radius(self, speed: float) -> float:
        """Calculate minimum turning radius for given speed.
        
        Two physical limits, whichever is bigger wins:
        - Centripetal (lateral acceleration): r = v²/a (dominates at speed)
        - Mechanical (steering geometry): r >= MIN_MECHANICAL_RADIUS_M
          (dominates at low speed — v²/a would otherwise shrink toward
          zero, which no real car's steering can achieve)
        
        Args:
            speed: Speed in m/s
        
        Returns:
            Minimum turning radius in meters
        """
        centripetal_radius = (speed ** 2) / self.max_lateral_accel
        return max(self.MIN_MECHANICAL_RADIUS_M, centripetal_radius)
    
    def plan_turn(
        self, 
        car_x: float, 
        car_y: float, 
        car_speed: float,
        car_heading: float,
        from_seg_idx: int,
        to_seg_idx: int,
        junction_node: str,
        network
    ) -> Optional[TurnPlan]:
        """Plan a circular arc turn between two road segments.
        
        Args:
            car_x, car_y: Current car position (world pixels)
            car_speed: Current speed (m/s)
            car_heading: Current heading (degrees)
            from_seg_idx: Current segment index
            to_seg_idx: Target segment index
            junction_node: Junction node ID
            network: RoadNetwork instance
        
        Returns:
            TurnPlan if turn is possible and stays on road, None otherwise
        """
        if from_seg_idx == to_seg_idx:
            return None
        
        from_seg = network.segments[from_seg_idx]
        to_seg = network.segments[to_seg_idx]
        
        # Get junction position
        if from_seg.end_node == junction_node:
            junction_x, junction_y = from_seg.x2, from_seg.y2
            from_dx = from_seg.x2 - from_seg.x1
            from_dy = from_seg.y2 - from_seg.y1
        else:
            junction_x, junction_y = from_seg.x1, from_seg.y1
            from_dx = from_seg.x1 - from_seg.x2
            from_dy = from_seg.y1 - from_seg.y2
        
        if to_seg.start_node == junction_node:
            to_dx = to_seg.x2 - to_seg.x1
            to_dy = to_seg.y2 - to_seg.y1
        else:
            to_dx = to_seg.x1 - to_seg.x2
            to_dy = to_seg.y1 - to_seg.y2
        
        from_heading = math.atan2(from_dx, from_dy)
        to_heading = math.atan2(to_dx, to_dy)
        
        # Calculate turn angle
        angle_diff = (to_heading - from_heading) % (2 * math.pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        
        # Determine turn direction
        clockwise = angle_diff < 0
        turn_angle = abs(angle_diff)
        
        # For very small turns (<10°), just do instant transition
        if turn_angle < math.radians(10):
            return None
        
        # Calculate required radius from speed
        required_radius = self.calculate_turning_radius(car_speed)
        
        # Geometric sanity cap: an arc can never plausibly fit if its radius
        # is large relative to the roads it connects (e.g. a 160m-radius arc
        # on 200m-long residential segments). Without this cap, the coarse
        # point-sampling in validate_arc_on_road() can produce false
        # positives for huge arcs that happen to graze close to a segment's
        # line for part of their length — i.e. a 90° turn at 120 km/h could
        # otherwise "validate" when it clearly shouldn't. This forces such
        # turns to be correctly rejected (caller then brakes and retries).
        max_sane_radius = 0.6 * min(from_seg.length, to_seg.length)
        if required_radius > max_sane_radius:
            print(f"\n🔍 Planning turn {from_seg_idx} → {to_seg_idx}, angle={math.degrees(turn_angle):.1f}°, "
                  f"speed={car_speed * 3.6:.0f} km/h")
            print(f"  ❌ Required radius {required_radius:.1f}m exceeds geometric cap "
                  f"{max_sane_radius:.1f}m (too fast for these road lengths) - rejecting")
            return None
        
        # Try multiple radii to find one that fits
        pppm = config.PIXELS_PER_METER
        
        print(f"\n🔍 Planning turn {from_seg_idx} → {to_seg_idx}, angle={math.degrees(turn_angle):.1f}°, speed={car_speed * 3.6:.0f} km/h")
        
        for radius_factor in [1.0, 1.2, 1.5, 2.0, 2.5]:
            test_radius = required_radius * radius_factor
            if test_radius > max_sane_radius:
                print(f"  ⏭️  Skipping radius {test_radius:.1f}m (factor {radius_factor}): exceeds geometric cap {max_sane_radius:.1f}m")
                continue
            print(f"  Trying radius: {test_radius:.1f}m (factor {radius_factor})")
            
            # Calculate arc center
            # IMPORTANT: anchor the START of the arc at the car's ACTUAL
            # current position and heading — not an idealized point on the
            # segment centerline computed from the junction. The car isn't
            # necessarily exactly `radius` meters before the junction, and
            # it may have a lane offset from the centerline; anchoring to
            # the idealized point caused a visible position jump (and
            # occasionally an off-road pop) at the instant the arc began.
            offset_angle = math.radians(car_heading) + (math.pi / 2 if not clockwise else -math.pi / 2)
            center_x = car_x + test_radius * pppm * math.sin(offset_angle)
            center_y = car_y - test_radius * pppm * math.cos(offset_angle)
            
            # Start angle: car's real current position around this center
            start_angle = math.atan2(car_y - center_y, car_x - center_x)
            
            # End angle: DIRECTLY offset from start_angle by the known,
            # exact turn_angle (not re-derived via a separate point/atan2
            # calculation). The previous approach computed end_angle
            # independently from a point projected along the to_seg
            # direction, which does NOT guarantee the swept angle equals
            # the intended turn_angle — for small radii especially, this
            # could accidentally require sweeping ~270° the "wrong way"
            # around the circle to reach that independently-computed point,
            # producing a near-full loop instead of a clean quarter-turn
            # (visibly confirmed via breadcrumb trail: the car looped
            # almost 360° right at the corner before merging onto the
            # road). Using start_angle +/- turn_angle guarantees the arc
            # sweeps EXACTLY the intended angle, every time.
            if clockwise:
                end_angle = start_angle - turn_angle
            else:
                end_angle = start_angle + turn_angle
            
            # Arc length now follows directly from the guaranteed sweep
            arc_angle = turn_angle
            arc_length = test_radius * arc_angle
            
            # Start and end points
            start_x = center_x + test_radius * pppm * math.cos(start_angle)
            start_y = center_y + test_radius * pppm * math.sin(start_angle)
            end_x = center_x + test_radius * pppm * math.cos(end_angle)
            end_y = center_y + test_radius * pppm * math.sin(end_angle)
            
            # Calibrate heading_offset so the arc's tangent heading at
            # progress=0 exactly matches the car's real current heading
            # (see TurnPlan.heading_offset docstring)
            raw_heading_at_0 = math.degrees(
                start_angle - math.pi / 2 if clockwise else start_angle + math.pi / 2
            )
            heading_offset = (car_heading - raw_heading_at_0) % 360

            # Create candidate turn plan
            candidate = TurnPlan(
                center_x=center_x,
                center_y=center_y,
                radius=test_radius,
                start_angle=start_angle,
                end_angle=end_angle,
                clockwise=clockwise,
                from_seg_idx=from_seg_idx,
                to_seg_idx=to_seg_idx,
                junction_node=junction_node,
                arc_length=arc_length,
                start_x=start_x,
                start_y=start_y,
                end_x=end_x,
                end_y=end_y,
                progress=0.0,
                heading_offset=heading_offset,
            )
            
            # VALIDATE: Check if entire arc stays on road
            if self.validate_arc_on_road(candidate, network, num_samples=20, debug=True):
                print(f"  ✅ Arc validated! Using radius {test_radius:.1f}m")
                return candidate
            else:
                print(f"  ❌ Arc validation failed at radius {test_radius:.1f}m")
        
        # No valid arc found
        print(f"  ⚠️ No valid arc found after trying all radii")
        return None
    
    def check_turn_feasible(
        self,
        car_speed: float,
        car_progress: float,
        current_seg_length: float,
        turn_plan: TurnPlan,
        forward: bool
    ) -> dict:
        """Check if turn is physically possible given current state.
        
        Args:
            car_speed: Current speed (m/s)
            car_progress: Progress on current segment (0.0 to 1.0)
            current_seg_length: Length of current segment (meters)
            turn_plan: Planned turn geometry
            forward: Direction on current segment
        
        Returns:
            dict with:
                feasible: bool - Can we make this turn?
                braking_required: bool - Do we need to brake?
                required_speed: float - Speed needed to fit turn radius (m/s)
                distance_to_brake: float - Distance available (meters)
        """
        # Calculate required speed for this turn's radius
        required_speed = math.sqrt(self.max_lateral_accel * turn_plan.radius)
        
        # Distance remaining to junction
        if forward:
            remaining_distance = current_seg_length * (1.0 - car_progress)
        else:
            remaining_distance = current_seg_length * car_progress
        
        # Check if we need to brake
        if car_speed <= required_speed:
            return {
                'feasible': True,
                'braking_required': False,
                'required_speed': required_speed,
                'distance_to_brake': remaining_distance
            }
        
        # Calculate braking distance: s = (v₁² - v₂²) / (2a)
        braking_distance = (car_speed ** 2 - required_speed ** 2) / (2 * config.CAR_BRAKING)
        safety_margin = 10.0  # meters
        
        # Can we brake in time?
        feasible = remaining_distance >= braking_distance + safety_margin
        
        return {
            'feasible': feasible,
            'braking_required': True,
            'required_speed': required_speed,
            'distance_to_brake': remaining_distance,
            'braking_distance_needed': braking_distance + safety_margin
        }
    
    def execute_turn(self, turn_plan: TurnPlan, distance_moved_m: float) -> Tuple[float, float, float, float]:
        """Advance along turn arc by given distance.
        
        Args:
            turn_plan: Current turn being executed
            distance_moved_m: Distance to move in meters
        
        Returns:
            (x, y, heading, new_progress)
        """
        # Calculate progress increment
        if turn_plan.arc_length > 0:
            progress_delta = distance_moved_m / turn_plan.arc_length
        else:
            progress_delta = 1.0
        
        new_progress = min(1.0, turn_plan.progress + progress_delta)
        
        # Get position and heading at new progress
        x, y, heading = turn_plan.get_point_on_arc(new_progress)
        
        return x, y, heading, new_progress
    
    def validate_arc_on_road(self, turn_plan: TurnPlan, network, num_samples: int = 10, debug: bool = False) -> bool:
        """Check if entire arc stays within road boundaries.
        
        Args:
            turn_plan: Turn to validate
            network: RoadNetwork instance
            num_samples: Number of points to check along arc
            debug: Print debug info for failed points
        
        Returns:
            True if entire arc is on road, False otherwise
        """
        pppm = config.PIXELS_PER_METER
        
        # Get the two segments we're transitioning between
        from_seg = network.segments[turn_plan.from_seg_idx]
        to_seg = network.segments[turn_plan.to_seg_idx]
        
        # Use generous buffer at junctions (road width + extra margin)
        # No extra buffer: match the ACTUAL rendered road width exactly.
        # A generous margin here was letting arcs validate that visibly
        # clipped the grass at the corner (confirmed via breadcrumb trail
        # screenshot) — the renderer draws roads at exactly half-width
        # with no slack, so validation must match that precisely.
        buffer_margin = 0.0
        
        failed_points = []
        
        for i in range(num_samples + 1):
            progress = i / num_samples
            x, y, _ = turn_plan.get_point_on_arc(progress)
            
            # Check if point is near EITHER from_seg OR to_seg with generous buffer
            on_road = False
            
            for seg in [from_seg, to_seg]:
                # Use generous buffer = half width + margin
                buffer = (seg.width / 2 + buffer_margin) * pppm
                
                # Point-to-segment distance
                dx = seg.x2 - seg.x1
                dy = seg.y2 - seg.y1
                length_sq = dx * dx + dy * dy
                if length_sq == 0:
                    continue
                
                # Project point onto segment
                t = max(0, min(1, ((x - seg.x1) * dx + (y - seg.y1) * dy) / length_sq))
                proj_x = seg.x1 + t * dx
                proj_y = seg.y1 + t * dy
                dist = math.hypot(x - proj_x, y - proj_y)
                
                if dist <= buffer:
                    on_road = True
                    break
            
            if not on_road:
                failed_points.append((i, progress, x, y))
        
        if failed_points and debug:
            print(f"    🚧 Arc validation failed at {len(failed_points)}/{num_samples+1} points:")
            for idx, prog, px, py in failed_points[:3]:  # Show first 3
                print(f"      Point {idx} (progress={prog:.2f}): ({px:.0f}, {py:.0f})")
        
        return len(failed_points) == 0
