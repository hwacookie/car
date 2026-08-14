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
        
        # Heading (tangent to circle)
        if self.clockwise:
            heading = math.degrees(current_angle - math.pi / 2)
        else:
            heading = math.degrees(current_angle + math.pi / 2)
        
        return x, y, heading % 360


class TurningSystem:
    """Calculates and manages physics-based circular arc turns."""
    
    def __init__(self, max_lateral_accel: float = 5.0):
        """Initialize turning system.
        
        Args:
            max_lateral_accel: Maximum lateral acceleration in m/s² (design constraint)
        """
        self.max_lateral_accel = max_lateral_accel
    
    def calculate_turning_radius(self, speed: float) -> float:
        """Calculate minimum turning radius for given speed.
        
        Based on centripetal acceleration: a = v²/r
        Therefore: r = v²/a
        
        Args:
            speed: Speed in m/s
        
        Returns:
            Minimum turning radius in meters
        """
        if speed < 0.1:
            return 1.0  # Minimum radius at very low speeds
        
        return (speed ** 2) / self.max_lateral_accel
    
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
        
        # Try multiple radii to find one that fits
        pppm = config.PIXELS_PER_METER
        
        for radius_factor in [1.0, 1.2, 1.5, 2.0, 2.5]:
            test_radius = required_radius * radius_factor
            
            # Calculate arc center
            # Center is perpendicular to the FROM segment at the junction
            offset_angle = from_heading + (math.pi / 2 if not clockwise else -math.pi / 2)
            center_x = junction_x + test_radius * pppm * math.sin(offset_angle)
            center_y = junction_y - test_radius * pppm * math.cos(offset_angle)
            
            # Calculate start and end angles (from center)
            start_angle = math.atan2(junction_y - center_y, junction_x - center_x)
            end_angle = math.atan2(
                junction_y + to_dy * test_radius * pppm / math.hypot(to_dx, to_dy) - center_y,
                junction_x + to_dx * test_radius * pppm / math.hypot(to_dx, to_dy) - center_x
            )
            
            # Adjust angles for proper arc direction
            if not clockwise:
                # Counter-clockwise
                while end_angle < start_angle:
                    end_angle += 2 * math.pi
            else:
                # Clockwise
                while end_angle > start_angle:
                    end_angle -= 2 * math.pi
            
            # Calculate arc length
            arc_angle = abs(end_angle - start_angle)
            arc_length = test_radius * arc_angle
            
            # Start and end points
            start_x = center_x + test_radius * pppm * math.cos(start_angle)
            start_y = center_y + test_radius * pppm * math.sin(start_angle)
            end_x = center_x + test_radius * pppm * math.cos(end_angle)
            end_y = center_y + test_radius * pppm * math.sin(end_angle)
            
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
                progress=0.0
            )
            
            # VALIDATE: Check if entire arc stays on road
            if self.validate_arc_on_road(candidate, network, num_samples=20):
                return candidate
        
        # No valid arc found
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
    
    def validate_arc_on_road(self, turn_plan: TurnPlan, network, num_samples: int = 10) -> bool:
        """Check if entire arc stays within road boundaries.
        
        Args:
            turn_plan: Turn to validate
            network: RoadNetwork instance
            num_samples: Number of points to check along arc
        
        Returns:
            True if entire arc is on road, False otherwise
        """
        pppm = config.PIXELS_PER_METER
        
        for i in range(num_samples + 1):
            progress = i / num_samples
            x, y, _ = turn_plan.get_point_on_arc(progress)
            
            # Check if this point is on either the from or to segment
            on_road = False
            
            for seg_idx in [turn_plan.from_seg_idx, turn_plan.to_seg_idx]:
                seg = network.segments[seg_idx]
                half_width = (seg.width / 2) * pppm
                
                # Point-to-segment distance
                dx = seg.x2 - seg.x1
                dy = seg.y2 - seg.y1
                length_sq = dx * dx + dy * dy
                if length_sq == 0:
                    continue
                
                t = max(0, min(1, ((x - seg.x1) * dx + (y - seg.y1) * dy) / length_sq))
                proj_x = seg.x1 + t * dx
                proj_y = seg.y1 + t * dy
                dist = math.hypot(x - proj_x, y - proj_y)
                
                if dist <= half_width:
                    on_road = True
                    break
            
            if not on_road:
                return False
        
        return True
