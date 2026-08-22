# Car Class - Pure Physics and State
# No input handling - controlled by Driver classes

from __future__ import annotations

import math
import pygame

from . import config


class Car:
    """A car with physics, state, and rendering. Controlled by a Driver."""
    
    # Class variable: cached sprite images
    _sprite_cache = {}

    # Monotonic per-instance identity. NOT id(): CPython reuses addresses,
    # so a freshly created Car can land on the freed address of the one it
    # replaced. Anything keying per-car state on id() then applies the dead
    # car's state to the new one - the physics validator did exactly that
    # and reported the new car's spawn as a 4.8 m jump at 0 m/s.
    _next_uid = 0
    
    def __init__(self, x: float, y: float, heading: float, seg_idx: int, driver=None):
        """x, y in world pixels. heading in degrees (0 = up/north)."""
        Car._next_uid += 1
        self.uid = Car._next_uid
        # Position and orientation
        self.x = x
        self.y = y
        self.heading = heading
        
        # Speed
        self.speed = 0.0  # m/s
        self.target_speed = 0.0
        
        # Road following state (segment index + progress along it)
        self.seg_idx = seg_idx
        self.progress = 0.5  # 0.0 to 1.0 along segment
        self.forward = True  # Direction on segment
        
        # Driver controlling this car
        self.driver = driver
        
        # Bicycle-model navigation (created lazily on first use - see update())
        self.bicycle_nav = None
        
        # Visual state
        self._braking = False
        self._accelerating = False
        
        # Debug trail
        self.trail = []
        self._trail_timer = 0.0
        self.trail_enabled = False
    
    # --- Main Update ---
    
    def update(self, dt: float, network, control_input: dict):
        """Update car physics based on control input from driver.
        
        control_input dict:
            accelerate: bool
            brake: bool
            steer_left: bool (FREE mode only)
            steer_right: bool (FREE mode only)
            blinker_left: bool (BICYCLE mode, from driver state)
            blinker_right: bool (BICYCLE mode, from driver state)
        """
        old_x, old_y = self.x, self.y
        
        # Update visual state
        self._braking = control_input.get('brake', False)
        self._accelerating = control_input.get('accelerate', False)
        
        # Choose update mode based on driver type
        name = self.driver.get_name() if self.driver else "FREE"
        if name == "BICYCLE":
            if self.bicycle_nav is None:
                from .bicycle_nav import BicycleNav
                self.bicycle_nav = BicycleNav(self, network)
            self.bicycle_nav.update(dt, control_input)
        else:
            self._update_free_mode(dt, control_input)
        
        # Update trail
        if self.trail_enabled:
            self._trail_timer += dt
            if self._trail_timer >= 0.1:
                self._trail_timer = 0.0
                self.trail.append((self.x, self.y, self.heading))
                if len(self.trail) > 500:
                    self.trail.pop(0)
    
    # --- FREE Mode Physics ---
    
    def _update_free_mode(self, dt: float, control_input: dict):
        """Manual steering mode with cruise control."""
        accel = control_input.get('accelerate', False)
        brake = control_input.get('brake', False)
        steer_left = control_input.get('steer_left', False)
        steer_right = control_input.get('steer_right', False)
        
        # Speed control
        if accel:
            self.speed += config.CAR_ACCELERATION * dt
        elif brake:
            self.speed -= config.CAR_BRAKING * dt
        
        self.speed = max(0, min(config.CAR_SPEED, self.speed))
        self.target_speed = self.speed
        
        # Steering (only when moving)
        if self.speed > 0:
            turn_factor = max(0.3, 1.0 - self.speed / config.CAR_SPEED * 0.7)
            turn_rate = config.CAR_TURN_SPEED * turn_factor * dt
            if steer_left:
                self.heading -= turn_rate
            if steer_right:
                self.heading += turn_rate
            self.heading = self.heading % 360
        
        # Movement
        rad = math.radians(self.heading)
        dx = math.sin(rad) * self.speed * dt * config.PIXELS_PER_METER
        dy = math.cos(rad) * self.speed * dt * config.PIXELS_PER_METER
        self.x += dx
        self.y += dy
    
    # --- Utility ---
    
    def snap_to_road(self, network):
        """Snap car to nearest road segment."""
        pppm = config.PIXELS_PER_METER
        best_seg = 0
        best_dist = float("inf")
        best_t = 0.5
        
        for idx, seg in enumerate(network.segments):
            dx = seg.x2 - seg.x1
            dy = seg.y2 - seg.y1
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                continue
            t = max(0, min(1, ((self.x - seg.x1) * dx + (self.y - seg.y1) * dy) / length_sq))
            proj_x = seg.x1 + t * dx
            proj_y = seg.y1 + t * dy
            dist = math.hypot(self.x - proj_x, self.y - proj_y)
            if dist < best_dist:
                best_dist = dist
                best_seg = idx
                best_t = t
        
        self.seg_idx = best_seg
        self.progress = best_t
        seg = network.segments[best_seg]
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_heading = math.degrees(math.atan2(dx, dy))
        diff = abs((self.heading - seg_heading + 180) % 360 - 180)
        self.forward = diff < 90
    
    def teleport_random(self, network):
        """Teleport to random road location.
        
        Note: Caller should notify PhysicsValidator.skip_next_frames() if using validation.
        """
        rx, ry, rh, seg_idx, node_id = network.random_road_point()
        self.x = rx
        self.y = ry
        self.heading = rh
        self.seg_idx = seg_idx
        self.progress = 0.5
        self.speed = 0
        self.target_speed = 0
        
        # Determine forward direction
        seg = network.segments[seg_idx]
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_heading = math.degrees(math.atan2(dx, dy))
        diff = abs((self.heading - seg_heading + 180) % 360 - 180)
        self.forward = diff < 90
        
        if self.bicycle_nav is not None:
            self.bicycle_nav.reset()
        
        # Clear trail
        self.trail.clear()

    def teleport_to_named_point(self, network, name: str):
        """Teleport to a deterministic named start point (synthetic test
        maps only — see RoadNetwork.start_points / test_maps.py).

        Note: Caller should notify PhysicsValidator.reset_car_state() if
        using validation.
        """
        x, y, heading, seg_idx, forward = network.get_start_point(name)
        self.x = x
        self.y = y
        self.heading = heading
        self.seg_idx = seg_idx
        self.progress = 0.0 if forward else 1.0
        self.forward = forward
        self.speed = 0
        self.target_speed = 0
        if self.bicycle_nav is not None:
            self.bicycle_nav.reset()
        self.trail.clear()

    def body_center(self) -> tuple[float, float]:
        """Geometric centre of the car body, in world pixels.

        self.x / self.y are the REAR AXLE (the bicycle model's pivot), so
        the body sits config.REAR_AXLE_OFFSET_M ahead of them along the
        heading. Anything that draws or measures the car's BODY (sprite,
        four-corner on-road box, blinkers) must use this, not (x, y).
        """
        rad = math.radians(self.heading)
        off = config.REAR_AXLE_OFFSET_M * config.PIXELS_PER_METER
        return self.x + math.sin(rad) * off, self.y + math.cos(rad) * off

    def is_on_road(self, network) -> bool:
        """Check if the car (all four corners, not just its center) is on
        any road.

        Delegates to RoadNetwork.is_car_on_road(), which tests the car's
        four bounding-box corners against the exact same paved-area polygon
        that gets rendered (rounded bends and junction fillets included).
        """
        bx, by = self.body_center()
        return network.is_car_on_road(bx, by, self.heading)
    
    # --- Rendering ---
    
    def draw(self, surface: pygame.Surface, camera):
        """Draw the car sprite."""
        # The sprite is the BODY, which sits ahead of the rear axle that
        # (self.x, self.y) tracks - see Car.body_center().
        sx, sy = camera.world_to_screen(*self.body_center())
        sx, sy = int(sx), int(sy)
        scale = camera.zoom
        
        # Calculate desired size in screen pixels
        # CAR_LENGTH and CAR_WIDTH are in meters
        car_length_px = config.CAR_LENGTH * config.PIXELS_PER_METER * scale
        car_width_px = config.CAR_WIDTH * config.PIXELS_PER_METER * scale
        
        # Load base sprite (high-res version)
        base_sprite = self._get_base_sprite()
        
        # Scale sprite to match world dimensions
        scaled_sprite = pygame.transform.scale(base_sprite, (int(car_width_px), int(car_length_px)))
        
        # Rotate sprite
        rotated = pygame.transform.rotate(scaled_sprite, -self.heading)
        
        # Center and draw
        rect = rotated.get_rect(center=(sx, sy))
        surface.blit(rotated, rect)
        
        # Draw dynamic lights on top (blinkers)
        if self.driver and self.driver.get_name() == "BICYCLE":
            self._draw_blinkers(surface, sx, sy, scale)
    
    def _get_base_sprite(self) -> pygame.Surface:
        """Load and cache the base car sprite (high-res version)."""
        import os
        from PIL import Image
        
        sprite_file = 'car_64x128.png'  # Use high-res version as base
        
        # Check cache
        if sprite_file not in Car._sprite_cache:
            # Load sprite using PIL (pygame doesn't have PNG support)
            sprite_path = os.path.join(os.path.dirname(__file__), '..', 'assets', sprite_file)
            if os.path.exists(sprite_path):
                # Load with PIL
                pil_image = Image.open(sprite_path).convert('RGBA')
                # Convert PIL image to pygame surface
                mode = pil_image.mode
                size = pil_image.size
                data = pil_image.tobytes()
                surf = pygame.image.fromstring(data, size, mode)
                Car._sprite_cache[sprite_file] = surf
            else:
                # Fallback: create simple colored rectangle if sprite not found
                size = (64, 128)
                surf = pygame.Surface(size, pygame.SRCALPHA)
                pygame.draw.rect(surf, (180, 30, 30), surf.get_rect())
                Car._sprite_cache[sprite_file] = surf
        
        return Car._sprite_cache[sprite_file]
    
    def _draw_blinkers(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        """Draw blinker lights (orange, flashing)."""
        if not self.driver or not hasattr(self.driver, 'blinker_left'):
            return
        
        if not (self.driver.blinker_left or self.driver.blinker_right):
            return
        
        # Flash with 0.5s period
        import time
        if (time.time() % 0.5) > 0.25:
            return
        
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        
        for side in ((-1, self.driver.blinker_left), (1, self.driver.blinker_right)):
            sign, active = side
            if not active:
                continue
            bx = sx + math.sin(rad) * config.CAR_LENGTH / 3 * config.PIXELS_PER_METER * scale
            bx += math.cos(rad) * sign * half_wid * 0.8
            by = sy - math.cos(rad) * config.CAR_LENGTH / 3 * config.PIXELS_PER_METER * scale
            by += math.sin(rad) * sign * half_wid * 0.8
            pygame.draw.circle(surface, (255, 180, 0), (int(bx), int(by)), 3)
