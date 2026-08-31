# Physics Validator - Independent "Physics Judge" for detecting violations

from __future__ import annotations
import math
from typing import Dict, Tuple
from . import config


class PhysicsValidator:
    """Independent physics constraint checker for cars.
    
    Detects violations:
    - Impossible position jumps (speed exceeds max)
    - Instant heading changes (>30° in one frame)
    - Rotating with an impossibly tight implied turning radius
    
    State is tracked by car id — when a car is destroyed and replaced,
    the old entry simply goes stale. No skip/reset plumbing needed.

    Violations are PER CAR (car_id -> log): each new car starts with an
    empty counter, which is what parallel test runs (one car per test in
    the same world) need.
    """
    
    HEADING_EPSILON_DEG = 0.05
    MIN_REALISTIC_RADIUS_M = 3.0
    
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        # car_id -> log of that car's violations
        self._violations_by_car: Dict[int, list] = {}
        # car_id -> (x, y, heading)
        self._last_state: Dict[int, Tuple[float, float, float]] = {}
    
    def violations_for(self, car) -> list:
        """This car's violation log (empty for a new/unknown car)."""
        return self._violations_by_car.get(getattr(car, "uid", id(car)), [])
    
    def count(self, car) -> int:
        """Number of violations logged for this car."""
        return len(self.violations_for(car))
    
    def enable(self):
        self.enabled = True
        print("✅ Physics validator ENABLED")
    
    def disable(self):
        self.enabled = False
        print("❌ Physics validator DISABLED")
    
    def check(self, car, dt: float, network, in_contact: bool = False):
        """Run all physics checks on a car. Called after car.update().
        
        in_contact: the car is pressed against an obstacle (stop-on-contact,
        docs/OBSTACLES.md). A contact stop is EXPECTED behavior, not a
        violation - but while resting against a solid object the motion is
        externally constrained (like being pushed by a truck), so the
        implied-turning-radius invariant is suspended for that frame. The
        jump / heading-snap / off-road checks keep running: the stop itself
        must stay physical (no teleport, no instant snap, no penetration).
        """
        if not self.enabled:
            return
        
        car_id = getattr(car, "uid", id(car))
        
        if car_id not in self._last_state:
            self._last_state[car_id] = (car.x, car.y, car.heading)
            return
        
        old_x, old_y, old_heading = self._last_state[car_id]
        
        self._check_jump(car, old_x, old_y, dt)
        self._check_heading_snap(car, old_heading)
        if not in_contact:
            self._check_turning_radius(car, old_x, old_y, old_heading)
        self._check_off_road(car, network)
        
        self._last_state[car_id] = (car.x, car.y, car.heading)
    
    def _check_jump(self, car, old_x: float, old_y: float, dt: float):
        """Impossible position jump."""
        distance_m = math.hypot(car.x - old_x, car.y - old_y) / config.PIXELS_PER_METER
        # abs(): speed is signed (negative = reverse gear); the bound is on
        # how far the car can travel per frame in EITHER direction.
        max_allowed = abs(car.speed) * dt + 0.1 * dt + 0.01
        
        if distance_m > max_allowed:
            import traceback
            msg = (
                f"\n{'='*70}\n"
                f"⚠️  IMPOSSIBLE JUMP!\n"
                f"{'='*70}\n"
                f"Old: ({old_x:.1f}, {old_y:.1f}) → New: ({car.x:.1f}, {car.y:.1f})\n"
                f"Distance: {distance_m:.1f}m (max: {max_allowed:.1f}m)\n"
                f"Speed: {car.speed:.1f} m/s | Segment: {car.seg_idx}\n"
                f"{'='*70}\n"
            )
            print(msg)
            traceback.print_stack()
            raise RuntimeError(msg)
    
    def _check_heading_snap(self, car, old_heading: float):
        """Instant heading change."""
        diff = abs((car.heading - old_heading + 180) % 360 - 180)
        
        if diff > 30:
            import traceback
            msg = (
                f"\n{'='*70}\n"
                f"⚠️  INSTANT HEADING CHANGE!\n"
                f"{'='*70}\n"
                f"{old_heading:.1f}° → {car.heading:.1f}° (Δ{diff:.1f}°)\n"
                f"Speed: {car.speed:.1f} m/s | Segment: {car.seg_idx}\n"
                f"{'='*70}\n"
            )
            print(msg)
            traceback.print_stack()
    
    def _check_off_road(self, car, network):
        """Off-road driving."""
        if not car.is_on_road(network):
            import traceback
            msg = (
                f"\n{'='*70}\n"
                f"⚠️  OFF-ROAD!\n"
                f"{'='*70}\n"
                f"({car.x:.1f}, {car.y:.1f}) | Speed: {car.speed:.1f} m/s\n"
                f"Segment: {car.seg_idx}\n"
                f"{'='*70}\n"
            )
            print(msg)
            traceback.print_stack()
            car_id = getattr(car, "uid", id(car))
            self._violations_by_car.setdefault(car_id, []).append({
                'type': 'off_road',
                'car': car_id,
                'position': (car.x, car.y),
                'speed': car.speed,
                'segment': car.seg_idx,
            })
    
    def _check_turning_radius(self, car, old_x: float, old_y: float, old_heading: float):
        """Heading change requires proportional movement."""
        diff_deg = abs((car.heading - old_heading + 180) % 360 - 180)
        
        if diff_deg <= self.HEADING_EPSILON_DEG:
            return
        
        dist_m = math.hypot(car.x - old_x, car.y - old_y) / config.PIXELS_PER_METER
        radius_m = dist_m / math.radians(diff_deg)
        
        if radius_m < self.MIN_REALISTIC_RADIUS_M:
            import traceback
            msg = (
                f"\n{'='*70}\n"
                f"⚠️  IMPOSSIBLE TURNING RADIUS!\n"
                f"{'='*70}\n"
                f"Δheading: {diff_deg:.2f}° | Δpos: {dist_m*1000:.1f}mm\n"
                f"Implied radius: {radius_m:.3f}m (min: {self.MIN_REALISTIC_RADIUS_M}m)\n"
                f"Segment: {car.seg_idx}\n"
                f"{'='*70}\n"
            )
            print(msg)
            traceback.print_stack()
            raise RuntimeError(msg)
