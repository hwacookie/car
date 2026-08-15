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
    
    # Distance (meters) from the junction, back along the FROM road, to
    # the tangent point where this arc begins (and forward along the TO
    # road to where it ends — symmetric for a single-radius fillet). The
    # car must reach this point on its current segment before the arc can
    # start; see Car._check_and_plan_turn().
    tangent_distance_m: float = 0.0
    
    # Distance (meters), measured from the TRUE network junction node
    # (not the lane-offset-shifted effective junction used internally for
    # the arc's own geometry), to where the arc actually starts/ends along
    # each real segment. These are what Car._update_position_rails and
    # _advance_active_turn should use for "how far until/past the
    # junction" bookkeeping, since they're guaranteed consistent with
    # plain segment-following's own progress parametrization (unlike
    # tangent_distance_m, which can differ by the lane offset's along-
    # track component - see plan_turn()).
    from_tangent_offset_m: float = 0.0
    to_tangent_offset_m: float = 0.0
    
    # The speed (m/s) this plan's radius was chosen for. Decided ONCE when
    # the plan is created (see TurningSystem.decide_target_speed_for_turn)
    # and then FROZEN - the whole point of planning a turn in advance
    # (like a real driver would) rather than reactively recalculating the
    # required radius every frame from whatever speed the car currently
    # happens to be going (which made the radius, and therefore the
    # tangent point, a moving target while braking).
    target_speed_mps: float = 0.0
    
    # How far to the right of centerline (meters) this plan was built
    # from. Car._lane_offset_factor should reach exactly this fraction of
    # the car's normal lane offset by the time the tangent point is
    # reached - see Car._get_or_create_planned_turn(). 1.0x normal lane
    # offset means the whole turn stays within our own lane (no swerve
    # needed at all); smaller values mean progressively more of the
    # opposing lane is required.
    lane_offset_m: float = 0.0
    
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
        # heading at progress=0 (see heading_offset docstring above).
        #
        # IMPORTANT: our game heading uses a "compass bearing" convention
        # (0 deg = +y/"up", measured via atan2(dx, dy)), but this circle's
        # x/y are parametrized in the STANDARD math convention (angle
        # measured via atan2(y, x), 0 deg = +x axis). These are different
        # angular systems related by: compass = 90 deg - standard_angle.
        # The tangent-vector's STANDARD-MATH angle is current_angle -/+ 90
        # deg (cw/ccw); that must then be explicitly converted to compass
        # bearing with the "90 - x" step below — skipping that conversion
        # (i.e. treating the standard-math tangent angle as if it were
        # already a compass heading) silently produces headings that are
        # wrong by 90-180 deg in a way that only shows up partway through
        # the arc, not at progress=0 (confirmed numerically: start matched
        # after calibration, but end was still 180 deg off).
        if self.clockwise:
            standard_math_angle = current_angle - math.pi / 2
        else:
            standard_math_angle = current_angle + math.pi / 2
        heading = 90 - math.degrees(standard_math_angle)
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
    
    def decide_target_speed_for_turn(self, turn_angle_deg: float, cruise_speed_mps: float) -> float:
        """Decide, ONCE, the target speed a driver would choose for a turn
        of this severity - like a human sizing up a corner in advance
        ("this looks like a 90 degree corner, I'll take it at ~20 km/h")
        rather than reactively figuring it out while already braking.
        
        For gentle turns, the target speed may be at or above the current
        cruise speed - meaning no braking is needed at all (e.g. a shallow
        highway sweep), matching how a real driver wouldn't brake for
        every slight bend.
        
        Args:
            turn_angle_deg: How sharp the turn is (0-180 degrees)
            cruise_speed_mps: The speed the car would otherwise maintain
        
        Returns:
            Target speed in m/s for this specific turn
        """
        # Calibrated against real-world guidance for turning a small car
        # (~5.5m wall-to-wall turning circle) through an intersection: a
        # true right-angle corner is realistically taken at roughly
        # 10-15 km/h (tighter still, close to walking pace, if you can't
        # swing wide into the target road), NOT the ~40 km/h this used to
        # assume - that speed would need a ~25m turning radius, wildly
        # more than an ordinary street corner's curb radius provides,
        # which is exactly why the planner used to have to fall back
        # through slower/lane-shifted attempts and still end up visibly
        # cutting across into the centerline/opposing lane just to make
        # the (too-large) arc fit.
        if turn_angle_deg >= 90:
            safe_speed = 15 / 3.6
        elif turn_angle_deg >= 60:
            safe_speed = 25 / 3.6
        elif turn_angle_deg >= 30:
            safe_speed = 45 / 3.6
        else:
            # Gentle enough that no dedicated slow-down is needed
            safe_speed = cruise_speed_mps
        return min(cruise_speed_mps, safe_speed) if cruise_speed_mps > 0 else safe_speed
    
    def plan_turn(
        self,
        target_speed_mps: float,
        from_seg_idx: int,
        to_seg_idx: int,
        junction_node: str,
        network,
        lane_offset_m: float = 0.0,
    ) -> Optional[TurnPlan]:
        """Plan a circular arc turn between two road segments, ONCE, for a
        fixed target speed (see decide_target_speed_for_turn). The
        resulting geometry (radius, tangent points, arc) is frozen and
        does not change as the car's actual speed varies while
        approaching - only the BRAKING decision (are we there yet, do we
        need to slow down more) depends on current speed, not the plan
        itself. This is what makes the tangent point a fixed target
        instead of one that drifts every frame.
        
        Args:
            target_speed_mps: Fixed speed this turn's radius is chosen for
            from_seg_idx: Current segment index
            to_seg_idx: Target segment index
            junction_node: Junction node ID
            network: RoadNetwork instance
            lane_offset_m: How far to the RIGHT of centerline (matching
                Car._apply_plain_segment_position's convention) to build
                the fillet from. 0.0 = pure centerline (may require using
                the opposing lane for tight turns). A positive value tries
                to keep the whole arc within the car's own lane - the
                caller (Car._get_or_create_planned_turn) tries the full
                lane offset first and only reduces it if the arc doesn't
                validate, so gentle turns/curves never swerve toward
                centerline at all.
        
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
        
        # Compute the EFFECTIVE junction point for the requested lane
        # offset: instead of building the fillet from the true (centerline)
        # junction, build it from the intersection of the two roads' lane
        # lines (each shifted lane_offset_m to the right of centerline).
        # This lets a turn that comfortably fits within the car's own lane
        # be planned WITHOUT ever swerving toward centerline - matching
        # how a real driver only moves toward the middle of the road (or
        # opposing lane) when the corner genuinely requires the extra
        # width, not for gentle bends.
        pppm_tmp = config.PIXELS_PER_METER
        if abs(lane_offset_m) < 1e-9:
            eff_junction_x, eff_junction_y = junction_x, junction_y
        else:
            offset_px = lane_offset_m * pppm_tmp
            # Right-hand normal for a direction (dx, dy), matching
            # Car._apply_plain_segment_position's convention exactly:
            # nx=-dy/len, ny=dx/len; lane point = centerline - n*offset
            from_len_tmp = math.hypot(from_dx, from_dy)
            to_len_tmp = math.hypot(to_dx, to_dy)
            fnx, fny = -from_dy / from_len_tmp, from_dx / from_len_tmp
            tnx, tny = -to_dy / to_len_tmp, to_dx / to_len_tmp
            # A point on each road's shifted lane line (using the junction
            # as a reference point on the true centerline)
            p1x, p1y = junction_x - fnx * offset_px, junction_y - fny * offset_px
            p2x, p2y = junction_x - tnx * offset_px, junction_y - tny * offset_px
            # Intersect line (p1, direction from_dx/dy) with line (p2,
            # direction to_dx/dy) via standard 2D line intersection
            denom = from_dx * to_dy - from_dy * to_dx
            if abs(denom) < 1e-9:
                # Parallel roads (shouldn't happen for a real turn) - fall
                # back to the true junction
                eff_junction_x, eff_junction_y = junction_x, junction_y
            else:
                t = ((p2x - p1x) * to_dy - (p2y - p1y) * to_dx) / denom
                eff_junction_x = p1x + t * from_dx
                eff_junction_y = p1y + t * from_dy
        
        # Calculate required radius from the FIXED target speed (respects
        # both the centripetal-force limit and the mechanical minimum)
        required_radius = self.calculate_turning_radius(target_speed_mps)
        pppm = config.PIXELS_PER_METER
        
        print(f"\n🔍 Planning turn {from_seg_idx} → {to_seg_idx}, angle={math.degrees(turn_angle):.1f}°, "
              f"target_speed={target_speed_mps * 3.6:.0f} km/h")
        
        # --- Proper geometric fillet construction ---
        # Standard "circle inscribed tangent to two lines meeting at a
        # vertex" geometry. This is independent of the car's current lane
        # position/heading, unlike the earlier approach (which anchored
        # the arc to wherever the car instantaneously was, causing it to
        # drift progressively further from the road the more it swept).
        #
        # u = unit direction of travel on the FROM road, pointing INTO the
        #     junction. v = unit direction of travel on the TO road,
        #     pointing AWAY from the junction.
        # The wedge angle at the junction (between "back along the
        # incoming road" and "forward along the outgoing road") is
        # (pi - turn_angle). For a circle of radius R inscribed in that
        # wedge, tangent to both roads:
        #   tangent_distance t = R * tan(turn_angle / 2)   (from junction,
        #     along each road, to where the arc touches it)
        #   center_distance   d = R / cos(turn_angle / 2)  (from junction,
        #     along the angle bisector, to the circle's center)
        from_len = math.hypot(from_dx, from_dy)
        to_len = math.hypot(to_dx, to_dy)
        u_x, u_y = from_dx / from_len, from_dy / from_len
        v_x, v_y = to_dx / to_len, to_dy / to_len
        
        half_turn = turn_angle / 2
        tangent_distance_m = required_radius * math.tan(half_turn)
        center_distance_m = required_radius / math.cos(half_turn)
        
        # Geometric sanity cap: reject if the tangent point would fall
        # beyond the actual road length available on either side (can't
        # fillet with a bigger radius than the roads themselves allow).
        max_tangent_m = 0.9 * min(from_seg.length, to_seg.length)
        if tangent_distance_m > max_tangent_m:
            print(f"  ❌ Required tangent distance {tangent_distance_m:.1f}m exceeds available "
                  f"road length {max_tangent_m:.1f}m (too fast for these road lengths) - rejecting")
            return None
        
        # Tangent points: t meters back along the FROM road from the
        # EFFECTIVE junction (accounting for lane_offset_m), and t meters
        # forward along the TO road
        tangent1_x = eff_junction_x - tangent_distance_m * pppm * u_x
        tangent1_y = eff_junction_y - tangent_distance_m * pppm * u_y
        tangent2_x = eff_junction_x + tangent_distance_m * pppm * v_x
        tangent2_y = eff_junction_y + tangent_distance_m * pppm * v_y
        
        # IMPORTANT: tangent1/tangent2 lie on a line through the EFFECTIVE
        # junction (shifted from the true, network-node junction whenever
        # lane_offset_m != 0), so their along-track distance from the TRUE
        # junction is NOT simply tangent_distance_m - eff_junction can be
        # shifted along the road direction too, not just perpendicular to
        # it (confirmed numerically: for a 90 deg corner this shift was
        # ~0.9m, causing a matching position jump at the arc hand-off
        # since Car._apply_plain_segment_position parametrizes distance
        # from the TRUE junction). Fix: project tangent1/tangent2 onto the
        # actual segment lines and measure distance from the TRUE junction
        # directly, so this always matches what plain segment-following
        # computes, by construction.
        # Project tangent1 onto the FROM segment's own line (perpendicular
        # projection, ignoring any lateral/lane offset component) to get
        # its along-track parametric position, then convert to "distance
        # from the TRUE junction" - this is exactly what plain
        # segment-following's progress-based parametrization measures.
        fseg_dx, fseg_dy = from_seg.x2 - from_seg.x1, from_seg.y2 - from_seg.y1
        fseg_len_sq = fseg_dx * fseg_dx + fseg_dy * fseg_dy
        t1_proj = ((tangent1_x - from_seg.x1) * fseg_dx + (tangent1_y - from_seg.y1) * fseg_dy) / fseg_len_sq
        if from_seg.end_node == junction_node:
            from_tangent_offset_m = (1.0 - t1_proj) * from_seg.length
        else:
            from_tangent_offset_m = t1_proj * from_seg.length
        
        tseg_dx, tseg_dy = to_seg.x2 - to_seg.x1, to_seg.y2 - to_seg.y1
        tseg_len_sq = tseg_dx * tseg_dx + tseg_dy * tseg_dy
        t2_proj = ((tangent2_x - to_seg.x1) * tseg_dx + (tangent2_y - to_seg.y1) * tseg_dy) / tseg_len_sq
        if to_seg.start_node == junction_node:
            to_tangent_offset_m = t2_proj * to_seg.length
        else:
            to_tangent_offset_m = (1.0 - t2_proj) * to_seg.length
        
        # Center: along the bisector of (-u, v), at center_distance from
        # the effective junction
        bis_x, bis_y = (-u_x + v_x), (-u_y + v_y)
        bis_len = math.hypot(bis_x, bis_y)
        if bis_len < 1e-9:
            # Degenerate (180 deg turn) - bisector undefined
            return None
        bis_x, bis_y = bis_x / bis_len, bis_y / bis_len
        center_x = eff_junction_x + center_distance_m * pppm * bis_x
        center_y = eff_junction_y + center_distance_m * pppm * bis_y
        
        # Start/end angles directly from the (exactly-tangent) points.
        start_angle = math.atan2(tangent1_y - center_y, tangent1_x - center_x)
        true_end_angle = math.atan2(tangent2_y - center_y, tangent2_x - center_x)
        
        # IMPORTANT: derive the sweep direction/magnitude directly from
        # these two correctly-constructed angles — do NOT trust the
        # heading-space `clockwise` flag here. Our heading convention
        # (0°="up", dx=sin(h), dy=cos(h)) has different chirality than the
        # standard math-plane convention used for this circle
        # (x=r*cos(angle), y=r*sin(angle)), so "clockwise in heading-space"
        # does NOT necessarily match "increasing/decreasing angle in
        # standard-plane". Trusting it here previously produced an
        # end_angle exactly 180° away from the true tangent point
        # (confirmed numerically). The shortest signed angular difference
        # between the two REAL tangent-point angles is always correct.
        raw_diff = true_end_angle - start_angle
        while raw_diff > math.pi:
            raw_diff -= 2 * math.pi
        while raw_diff <= -math.pi:
            raw_diff += 2 * math.pi
        
        arc_clockwise = raw_diff < 0
        end_angle = start_angle + raw_diff
        arc_length = required_radius * abs(raw_diff)
        
        # Heading calibration: by construction the tangent direction at
        # the start point already matches the FROM road's own heading
        # exactly (that's what tangency means), so calibrate against the
        # ROAD's heading, not the car's instantaneous heading (which may
        # still be settling/approaching and isn't the calibration ground
        # truth here).
        from_heading_deg = math.degrees(from_heading) % 360
        standard_math_angle_at_0 = (
            start_angle - math.pi / 2 if arc_clockwise else start_angle + math.pi / 2
        )
        raw_heading_at_0 = 90 - math.degrees(standard_math_angle_at_0)  # compass conversion
        heading_offset = (from_heading_deg - raw_heading_at_0) % 360
        
        candidate = TurnPlan(
            center_x=center_x,
            center_y=center_y,
            radius=required_radius,
            start_angle=start_angle,
            end_angle=end_angle,
            clockwise=arc_clockwise,
            from_seg_idx=from_seg_idx,
            to_seg_idx=to_seg_idx,
            junction_node=junction_node,
            arc_length=arc_length,
            start_x=tangent1_x,
            start_y=tangent1_y,
            end_x=tangent2_x,
            end_y=tangent2_y,
            progress=0.0,
            tangent_distance_m=tangent_distance_m,
            from_tangent_offset_m=from_tangent_offset_m,
            to_tangent_offset_m=to_tangent_offset_m,
            target_speed_mps=target_speed_mps,
            lane_offset_m=lane_offset_m,
            heading_offset=heading_offset,
        )
        
        # Safety-net validation (should virtually always pass now, since
        # the fillet is tangent to both roads by construction - opposing
        # lane use is explicitly allowed, no lane-keeping constraint)
        if self.validate_arc_on_road(candidate, network, num_samples=20, debug=True):
            print(f"  ✅ Arc validated! radius={required_radius:.1f}m tangent_distance={tangent_distance_m:.1f}m")
            return candidate
        else:
            print(f"  ❌ Arc failed safety-net validation at radius {required_radius:.1f}m — unexpected, "
                  f"check road geometry")
            return None
    
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

        # Small buffer (not the earlier 5m one, which let arcs visibly
        # clip the grass): this covers the residual mismatch between the
        # arc's anchor (the car's current LANE position, offset ~1.75m
        # from centerline) and the idealized tangent point a perfect
        # geometric fillet would use. Real roads have comparable
        # curb/shoulder slack beyond the marked lane width, so this stays
        # physically reasonable while resolving marginal (~0.3m) rejections.
        buffer_margin_px = config.ROAD_EDGE_TOLERANCE_M * pppm

        # Test against the exact same paved-area polygon that gets
        # rendered (rounded bends and junction fillets included) instead
        # of an independent rectangle+circle approximation - this used to
        # be able to disagree with what's actually drawn/driveable.
        from shapely.geometry import Point
        paved = network.get_paved_polygon()

        failed_points = []

        for i in range(num_samples + 1):
            progress = i / num_samples
            x, y, _ = turn_plan.get_point_on_arc(progress)

            on_road = paved.distance(Point(x, y)) <= buffer_margin_px

            if not on_road:
                failed_points.append((i, progress, x, y))
        
        if failed_points and debug:
            print(f"    🚧 Arc validation failed at {len(failed_points)}/{num_samples+1} points:")
            for idx, prog, px, py in failed_points[:3]:  # Show first 3
                print(f"      Point {idx} (progress={prog:.2f}): ({px:.0f}, {py:.0f})")
        
        return len(failed_points) == 0
