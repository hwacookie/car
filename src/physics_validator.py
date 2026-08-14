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
    - Rotating with an impossibly tight implied turning radius (heading
      changes while the car barely moves — a car has a nonzero minimum
      turning radius, so distance_moved / angle_rotated can never drop
      below it; this is a hard invariant, checked every single frame)
    - Off-road driving (RAILS mode only)
    
    Can be enabled/disabled per car for performance.
    """
    
    # "Rotation requires proportional movement" tolerances
    HEADING_EPSILON_DEG = 0.05      # below this, no meaningful rotation to check
    # Kept comfortably below TurningSystem.MIN_MECHANICAL_RADIUS_M (5.0m)
    # so legitimate arcs never trip this by numerical/geometric slack,
    # while anything meaningfully tighter than a real car can achieve
    # still gets caught.
    MIN_REALISTIC_RADIUS_M = 3.0
    DISTANCE_EPSILON_M = 0.001       # floating-point noise floor for "didn't move at all"
    
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
        """Hard invariant: a car cannot change heading without moving a
        distance PROPORTIONAL to that rotation. It has a minimum turning
        radius r_min > 0, so rotating by angle theta necessarily sweeps at
        least an arc of length r_min*theta — there is no such thing as a
        car turning "on the spot", even a little bit each frame.
        
        My first version of this check only flagged EXACTLY zero movement
        (< 1mm), which missed the actual bug: while an arc is executing,
        distance = speed*dt is never exactly zero (even a few mm/frame at
        low speed), so heading rotating a lot while barely translating
        slipped through. The correct check is the IMPLIED radius
        (distance_moved / angle_rotated) — if that's smaller than any
        real car could achieve, the motion is impossible, regardless of
        whether distance_moved itself is nonzero.
        
        Checked directly every frame — no rolling windows, just the one
        physical constant (MIN_REALISTIC_RADIUS_M).
        """
        heading_diff_deg = abs((car.heading - old_heading + 180) % 360 - 180)
        distance_moved_m = math.hypot(car.x - old_x, car.y - old_y) / config.PIXELS_PER_METER
        
        if heading_diff_deg <= self.HEADING_EPSILON_DEG:
            return  # no meaningful rotation this frame, nothing to check
        
        heading_diff_rad = math.radians(heading_diff_deg)
        implied_radius_m = distance_moved_m / heading_diff_rad
        
        if implied_radius_m < self.MIN_REALISTIC_RADIUS_M:
            import traceback
            error_msg = (
                f"\n{'='*70}\n"
                f"⚠️  ROTATING WITH IMPOSSIBLE TURNING RADIUS!\n"
                f"{'='*70}\n"
                f"Heading changed by {heading_diff_deg:.2f}° while moving only "
                f"{distance_moved_m*1000:.1f}mm.\n"
                f"Implied turning radius: {implied_radius_m:.3f}m "
                f"(minimum realistic: {self.MIN_REALISTIC_RADIUS_M}m)\n"
                f"No real car can turn this tightly — position/heading updates\n"
                f"fell out of sync somewhere in the simulation (e.g. the car is\n"
                f"effectively spinning in place instead of sweeping a proper arc).\n"
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
