# Physics Validator - Independent "Physics Judge" for detecting violations
# Can be enabled/disabled per car for performance

from __future__ import annotations
import math
import time
from typing import Dict, Tuple, Optional
from . import config


class PhysicsValidator:
    """Independent physics constraint checker for cars.
    
    Detects violations:
    - Teleportation (position jumps)
    - Instant heading changes (>30° in one frame)
    - Rotating in place (heading changes without any position change —
      a car has a nonzero turning radius, so ANY rotation must be
      accompanied by translation; this is a hard invariant, checked
      every single frame, not a heuristic)
    - Off-road driving (RAILS mode only)
    
    Can be enabled/disabled per car for performance.
    """
    
    # "Rotation requires movement" tolerances (floating-point noise only)
    HEADING_EPSILON_DEG = 0.05
    DISTANCE_EPSILON_M = 0.001
    
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        # State tracking: car_id -> (x, y, heading, timestamp)
        self._last_state: Dict[int, Tuple[float, float, float, float]] = {}
        # Skip frames after intentional resets
        self._frames_to_skip: Dict[int, int] = {}
    
    def enable(self):
        """Enable validation checks."""
        self.enabled = True
        print("✅ Physics validator ENABLED")
    
    def disable(self):
        """Disable validation checks (for performance)."""
        self.enabled = False
        print("❌ Physics validator DISABLED")
    
    def skip_next_frames(self, car, num_frames: int = 5):
        """Skip validation for next N frames (after teleport, reset, etc)."""
        car_id = id(car)
        self._frames_to_skip[car_id] = num_frames
    
    def check(self, car, dt: float, network):
        """Run all physics checks on a car.
        
        Called from main loop AFTER car.update().
        """
        if not self.enabled:
            return
        
        car_id = id(car)
        
        # Skip frames if requested
        if car_id in self._frames_to_skip:
            if self._frames_to_skip[car_id] > 0:
                self._frames_to_skip[car_id] -= 1
                # Update state but don't check
                self._last_state[car_id] = (car.x, car.y, car.heading, time.time())
                return
            else:
                del self._frames_to_skip[car_id]
        
        # Get previous state
        if car_id not in self._last_state:
            # First check - just store state
            self._last_state[car_id] = (car.x, car.y, car.heading, time.time())
            return
        
        old_x, old_y, old_heading, old_time = self._last_state[car_id]
        
        # Check 1: Teleportation (position jump)
        self._check_teleportation(car, old_x, old_y, dt)
        
        # Check 2: Instant heading change
        if car.driver and car.driver.get_name() == "RAILS":
            self._check_heading_snap(car, old_heading)
        
        # Check 3: Rotating in place (heading changed but position didn't)
        # Hard invariant: a car has a nonzero turning radius, so it can
        # only be a spot-turn if it's not actually moving. Checked every
        # frame directly — no heuristics, no thresholds beyond float noise.
        if car.driver and car.driver.get_name() == "RAILS":
            self._check_rotation_requires_movement(car, old_x, old_y, old_heading)
        
        # Check 4: Off-road in RAILS mode
        if car.driver and car.driver.get_name() == "RAILS":
            self._check_off_road(car, network)
        
        # Update state
        self._last_state[car_id] = (car.x, car.y, car.heading, time.time())
    
    def _check_teleportation(self, car, old_x: float, old_y: float, dt: float):
        """Check for impossible position jumps."""
        dx = car.x - old_x
        dy = car.y - old_y
        distance_moved = math.hypot(dx, dy)
        pppm = config.PIXELS_PER_METER
        distance_m = distance_moved / pppm
        
        # Maximum allowed: speed * dt * safety factor + margin for segment transitions
        max_allowed_m = max(50, car.speed * dt * 3 + 20)
        
        if distance_m > max_allowed_m:
            import traceback
            error_msg = (
                f"\n{'='*70}\n"
                f"⚠️  TELEPORTATION DETECTED!\n"
                f"{'='*70}\n"
                f"Old position: ({old_x:.1f}, {old_y:.1f})\n"
                f"New position: ({car.x:.1f}, {car.y:.1f})\n"
                f"Distance: {distance_m:.1f}m (max allowed: {max_allowed_m:.1f}m)\n"
                f"Speed: {car.speed:.1f} m/s ({car.speed * 3.6:.0f} km/h)\n"
                f"Driver: {car.driver.get_name() if car.driver else 'None'}\n"
                f"Segment: {car.seg_idx}, Progress: {car.progress:.3f}\n"
                f"dt: {dt:.4f}s\n"
                f"{'='*70}\n"
            )
            print(error_msg)
            traceback.print_stack()
            raise RuntimeError(error_msg)
    
    def _check_heading_snap(self, car, old_heading: float):
        """Check for instant heading changes (should be smooth in RAILS mode)."""
        heading_diff = abs((car.heading - old_heading + 180) % 360 - 180)
        
        # Allow instant changes only during smooth transitions
        if heading_diff > 30 and not car._heading_transition:
            import traceback
            error_msg = (
                f"\n{'='*70}\n"
                f"⚠️  INSTANT HEADING CHANGE DETECTED!\n"
                f"{'='*70}\n"
                f"Old heading: {old_heading:.1f}°\n"
                f"New heading: {car.heading:.1f}°\n"
                f"Change: {heading_diff:.1f}° (max allowed: 30°)\n"
                f"Position: ({car.x:.1f}, {car.y:.1f})\n"
                f"Speed: {car.speed:.1f} m/s ({car.speed * 3.6:.0f} km/h)\n"
                f"Segment: {car.seg_idx}, Progress: {car.progress:.3f}\n"
                f"Driver: {car.driver.get_name()}\n"
                f"In transition: {car._heading_transition}\n"
                f"{'='*70}\n"
            )
            print(error_msg)
            traceback.print_stack()
            # Don't raise - just warn for now
    
    def _check_off_road(self, car, network):
        """Check if car is completely on road (RAILS mode constraint)."""
        # TODO: Implement proper 4-wheel check
        # For now, use center point check
        if not car.is_on_road(network):
            import traceback
            error_msg = (
                f"\n{'='*70}\n"
                f"⚠️  OFF-ROAD DETECTED (RAILS MODE)!\n"
                f"{'='*70}\n"
                f"Position: ({car.x:.1f}, {car.y:.1f})\n"
                f"Speed: {car.speed:.1f} m/s ({car.speed * 3.6:.0f} km/h)\n"
                f"Heading: {car.heading:.1f}°\n"
                f"Segment: {car.seg_idx}, Progress: {car.progress:.3f}\n"
                f"Driver: {car.driver.get_name()}\n"
                f"{'='*70}\n"
            )
            print(error_msg)
            traceback.print_stack()
            # Don't raise - just warn for now
    
    def _check_rotation_requires_movement(self, car, old_x: float, old_y: float, old_heading: float):
        """Hard invariant: a car cannot change heading without also
        changing position. It has a nonzero turning radius (r > 0), so
        rotating by any angle theta necessarily sweeps an arc of length
        r*theta > 0 — there is no such thing as a car turning "on the
        spot". If heading changed but position didn't, something in the
        simulation computed an impossible motion (e.g. a buggy arc whose
        position and heading updates got out of sync).
        
        Checked directly every frame — no thresholds beyond floating-point
        noise, no rolling windows, no heuristics.
        """
        heading_diff = abs((car.heading - old_heading + 180) % 360 - 180)
        distance_moved_m = math.hypot(car.x - old_x, car.y - old_y) / config.PIXELS_PER_METER
        
        if heading_diff > self.HEADING_EPSILON_DEG and distance_moved_m < self.DISTANCE_EPSILON_M:
            import traceback
            error_msg = (
                f"\n{'='*70}\n"
                f"⚠️  ROTATING IN PLACE DETECTED (impossible)!\n"
                f"{'='*70}\n"
                f"Heading changed by {heading_diff:.2f}° but position moved only "
                f"{distance_moved_m*1000:.2f}mm.\n"
                f"A car has a nonzero turning radius — it cannot rotate without\n"
                f"also translating. This means position/heading updates fell\n"
                f"out of sync somewhere in the simulation.\n"
                f"Old position: ({old_x:.1f}, {old_y:.1f}), heading {old_heading:.1f}°\n"
                f"New position: ({car.x:.1f}, {car.y:.1f}), heading {car.heading:.1f}°\n"
                f"Speed: {car.speed:.1f} m/s ({car.speed * 3.6:.0f} km/h)\n"
                f"Segment: {car.seg_idx}, Progress: {car.progress:.3f}\n"
                f"Driver: {car.driver.get_name()}\n"
                f"{'='*70}\n"
            )
            print(error_msg)
            traceback.print_stack()
            raise RuntimeError(error_msg)
    
    def reset_car_state(self, car):
        """Clear stored state for a car (after manual teleport, etc)."""
        car_id = id(car)
        if car_id in self._last_state:
            del self._last_state[car_id]
        self.skip_next_frames(car, 5)
