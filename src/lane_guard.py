# Lane Guard — detects wrong-side (oncoming lane) driving
# Softer than PhysicsValidator: warns and tracks stats, never crashes

from __future__ import annotations
import math
from typing import Dict, Tuple
from . import config


class LaneGuard:
    """Checks whether the car crosses into the opposing lane.

    On every two-way road segment the guard measures the distance from
    the car's center to the segment's centerline. If that distance drops
    below half the car's width, part of the car is on the wrong side of
    the centerline.

    Unlike PhysicsValidator this NEVER raises — it only logs warnings
    and accumulates statistics for test reports.
    """
    
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        
        # Per-car state: car_id -> last lateral offset (m)
        self._last_offset: Dict[int, float] = {}
        
        # Statistics
        self.violations: int = 0          # number of frames on wrong side
        self.violation_time: float = 0.0  # cumulative seconds on wrong side
    
    def enable(self):
        self.enabled = True
        print("✅ Lane guard ENABLED")
    
    def disable(self):
        self.enabled = False
        print("❌ Lane guard DISABLED")
    
    def check(self, car, dt: float, network) -> bool:
        """Return True if the car is currently on the wrong side."""
        if not self.enabled:
            return False
        
        seg = network.segments[car.seg_idx]
        
        # One-way streets have no opposing lane — skip silently
        if seg.oneway:
            return False
        
        offset_m = self._lateral_offset(car, seg, network)
        car_id = id(car)

        # If distance to centerline < half car width (0.9m), the left edge
        # crosses into the opposing lane.
        HALF_CAR_WIDTH = 0.9
        EPSILON = 0.02  # avoid false positives from floating-point noise at exact boundary
        was_wrong = car_id in self._last_offset and self._last_offset[car_id] < (HALF_CAR_WIDTH - EPSILON)
        is_wrong = offset_m < (HALF_CAR_WIDTH - EPSILON)
        
        if is_wrong:
            self.violations += 1
            self.violation_time += dt
            # Only print once per crossing event (not every frame)
            if not was_wrong:
                print(
                    f"\n⚠️  WRONG-SIDE DRIVING! distance to centerline "
                    f"{offset_m:.2f} m (half car width = 0.90 m) on segment {car.seg_idx}\n"
                )
        
        self._last_offset[car_id] = offset_m
        return is_wrong
    
    def _lateral_offset(self, car, seg, network) -> float:
        """Lateral distance (metres) from car center to the segment's centerline.

        Always positive — it's just how far off-center the car is.
        """
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        length_sq = dx * dx + dy * dy
        if length_sq == 0:
            return 0.0
        # Measure from the BODY centre, not car.x/y - those are the rear
        # axle (the bicycle model's pivot), which in a bend sits on a
        # different lateral offset than the body it is supposed to stand
        # for.
        cx, cy = car.body_center() if hasattr(car, "body_center") else (car.x, car.y)

        # Distance to the REAL centreline, which curves through bends -
        # not to the straight chord between the segment's endpoints. At a
        # rounded bend the two diverge by more than the margin being
        # measured, so the chord version reported violations for a car
        # sitting correctly in its lane (and would miss real ones).
        geom = self._centreline_geom(network)
        if geom is not None:
            from shapely.geometry import Point
            return geom.distance(Point(cx, cy)) / config.PIXELS_PER_METER

        t = max(0.0, min(1.0, ((cx - seg.x1) * dx + (cy - seg.y1) * dy) / length_sq))
        proj_x = seg.x1 + t * dx
        proj_y = seg.y1 + t * dy
        return math.hypot(cx - proj_x, cy - proj_y) / config.PIXELS_PER_METER

    @staticmethod
    def _centreline_geom(network):
        """Cached union of the two-way road centrelines (the same merged,
        corner-rounded lines the dashed markings are drawn from, so the
        guard tests exactly the line the player can see). One-way roads
        are absent by construction - they have no oncoming lane."""
        geom = getattr(network, "_laneguard_centrelines", None)
        if geom is None:
            from shapely.geometry import MultiLineString
            lines = [c for c in network.get_centerlines() if len(c) >= 2]
            geom = MultiLineString(lines) if lines else None
            network._laneguard_centrelines = geom
        return geom
    
    def reset(self, car):
        """Clear stored state after teleport."""
        car_id = id(car)
        self._last_offset.pop(car_id, None)
    
    def stats(self) -> dict:
        """Return a summary dict suitable for test reports."""
        return {
            "wrong_side_frames": self.violations,
            "wrong_side_seconds": round(self.violation_time, 2),
        }
