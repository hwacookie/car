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
    """
    
    HEADING_EPSILON_DEG = 0.05
    MIN_REALISTIC_RADIUS_M = 3.0
    
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.violations: list[dict] = []  # log of all violations
        # car_id -> (x, y, heading)
        self._last_state: Dict[int, Tuple[float, float, float]] = {}
    
    def enable(self):
        self.enabled = True
        print("✅ Physics validator ENABLED")
    
    def disable(self):
        self.enabled = False
        print("❌ Physics validator DISABLED")
    
    def check(self, car, dt: float, network):
        """Run all physics checks on a car. Called after car.update()."""
        if not self.enabled:
            return
        
        car_id = id(car)
        
        if car_id not in self._last_state:
            self._last_state[car_id] = (car.x, car.y, car.heading)
            return
        
        old_x, old_y, old_heading = self._last_state[car_id]
        
        self._check_jump(car, old_x, old_y, dt)
        self._check_heading_snap(car, old_heading)
        self._check_turning_radius(car, old_x, old_y, old_heading)
        self._check_off_road(car, network)
        
        self._last_state[car_id] = (car.x, car.y, car.heading)
    
    def _check_jump(self, car, old_x: float, old_y: float, dt: float):
        """Impossible position jump."""
        distance_m = math.hypot(car.x - old_x, car.y - old_y) / config.PIXELS_PER_METER
        max_allowed = car.speed * dt + 0.1 * dt + 0.01
        
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
            self.violations.append({
                'type': 'off_road',
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
