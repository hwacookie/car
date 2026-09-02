# Car Class - Pure Physics and State
# No input handling - controlled by Driver classes

from __future__ import annotations

import math

from . import config


class Car:
    """A car with physics and state. Controlled by a Driver.
    (Rendering lives in the remote frontend since M5.)"""

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
        # Display color (multi-car, docs/MULTI_CAR_PLAN.md): the main loop
        # assigns it deterministically from the uid; "red" is the player
        # default for cars created outside that loop.
        self.color = "red"
        # Position and orientation
        self.x = x
        self.y = y
        self.heading = heading
        
        # Speed
        self.speed = 0.0  # m/s
        self.target_speed = 0.0
        # Engaged gear (FREE mode, like a real shifter): None = neutral,
        # 'fwd' or 'rev'. Set by a FRESH key press at a standstill and
        # persists afterwards; each gear only responds to its own throttle
        # key (see Car._update_free_mode).
        self._gear = None
        # Virtual steering wheel position (FREE mode), -1..+1. Ramps toward
        # the demanded direction at a finite rate instead of jumping to
        # full lock on key-down (see config.STEER_LOCK_TIME_S).
        self._steer_pos = 0.0

        # Current steering angle in RADIANS, set by the navigation model
        # each physics step (+ = right). The driver's mechanical blinker
        # auto-off (real-car steering cam) reads this.
        self.steer_angle = 0.0

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
            self._update_free_mode(dt, control_input, network)
        
        # Update trail
        if self.trail_enabled:
            self._trail_timer += dt
            if self._trail_timer >= 0.1:
                self._trail_timer = 0.0
                self.trail.append((self.x, self.y, self.heading))
                if len(self.trail) > 500:
                    self.trail.pop(0)
    
    # --- FREE Mode Physics ---
    
    def _update_free_mode(self, dt: float, control_input: dict, network=None):
        """Manual steering mode with a real-car gear model.

        Speed is signed: positive = forward, negative = reverse. A real
        car must brake through zero before it can back up - you cannot
        jump from +v straight to -v:
          moving forward  : S brakes to a stop, W accelerates
          moving backward : W brakes to a stop, S accelerates (reverse)
          at a standstill : a FRESH press of S engages reverse, a fresh
                            press of W drives forward. Holding the brake
                            key through zero never shifts gears.
        """
        accel = control_input.get('accelerate', False)      # W held
        brake = control_input.get('brake', False)           # S held
        accel_pressed = control_input.get('accelerate_pressed', False)
        brake_pressed = control_input.get('brake_pressed', False)
        steer_left = control_input.get('steer_left', False)
        steer_right = control_input.get('steer_right', False)
        
        # Speed control (signed, explicit gear state like a real car):
        #   moving forward  : S brakes to a stop, W accelerates
        #   moving backward : W brakes to a stop, S accelerates (reverse)
        #   at a standstill : a FRESH press of S shifts into reverse, a
        #                     fresh press of W shifts into drive. The gear
        #                     then persists (like a shifter) and only its
        #                     own throttle key moves the car - so holding
        #                     the brake through zero never engages anything.
        if self.speed > 0:            # moving forward
            if brake:
                self.speed -= config.CAR_BRAKING * dt
            elif accel:
                self.speed += config.CAR_ACCELERATION * dt
            if self.speed < 0:        # braking ends exactly at zero
                self.speed = 0.0
        elif self.speed < 0:          # moving backward
            if accel:                 # W is the brake in reverse
                self.speed += config.CAR_BRAKING * dt
            elif brake:               # S is the throttle in reverse
                self.speed -= config.CAR_ACCELERATION * dt
            if self.speed > 0:        # ...and ends exactly at zero
                self.speed = 0.0
        else:                         # standstill: shifting + throttle
            if brake_pressed:
                self._gear = 'rev'
            elif accel_pressed:
                self._gear = 'fwd'
            if self._gear == 'rev' and brake:
                self.speed -= config.CAR_ACCELERATION * dt
            elif self._gear == 'fwd' and accel:
                self.speed += config.CAR_ACCELERATION * dt
        
        self.speed = max(-config.REVERSE_MAX_SPEED_M,
                         min(config.CAR_SPEED, self.speed))
        # Deadband so the car doesn't oscillate around 0 when neither
        # gear input is held.
        if not accel and not brake and abs(self.speed) < 0.05:
            self.speed = 0.0
        self.target_speed = self.speed

        # Brake lights: S while moving forward, W while reversing.
        self._braking = (self.speed > 0 and brake) or \
                        (self.speed < 0 and accel)
        
        # Steering (only when moving). The wheel position ramps toward the
        # demanded direction at a finite rate (config.STEER_LOCK_TIME_S)
        # and eases back to center on release - instant full lock on
        # key-down felt twitchy and "too direct". In reverse the yaw goes
        # the OTHER way for the same steering input (bicycle model:
        # omega = v*tan(d)/L with signed v) - that is how backing a car
        # into a spot works.
        steer_target = 0.0
        if steer_left:
            steer_target -= 1.0
        if steer_right:
            steer_target += 1.0
        wheel_rate = dt / config.STEER_LOCK_TIME_S
        if self._steer_pos < steer_target:
            self._steer_pos = min(steer_target, self._steer_pos + wheel_rate)
        elif self._steer_pos > steer_target:
            self._steer_pos = max(steer_target, self._steer_pos - wheel_rate)

        if abs(self.speed) > 0 and abs(self._steer_pos) > 1e-6:
            speed_abs = abs(self.speed)
            turn_factor = max(0.3, 1.0 - speed_abs / config.CAR_SPEED * 0.7)
            turn_rate = config.CAR_TURN_SPEED * turn_factor * dt
            # A real car's yaw rate at FULL lock is v / R_min (the bicycle
            # model): the fixed arcade rate above would imply a turning
            # radius below the mechanical minimum at low speed - physically
            # impossible, and exactly what the validator rejects. Cap the
            # per-frame heading change with the same limit BICYCLE mode has.
            max_rate = math.degrees(speed_abs / config.MIN_TURN_RADIUS_M) * dt
            # Partial lock scales the yaw proportionally (the cap above is
            # the full-lock value).
            turn_rate = min(turn_rate, max_rate) * abs(self._steer_pos)
            sign = 1.0 if self.speed >= 0 else -1.0
            self.heading = (self.heading
                            + self._steer_pos * sign * turn_rate) % 360
        
        # Movement (signed speed handles reverse)
        rad = math.radians(self.heading)
        dx = math.sin(rad) * self.speed * dt * config.PIXELS_PER_METER
        dy = math.cos(rad) * self.speed * dt * config.PIXELS_PER_METER
        self.x += dx
        self.y += dy
        
        # Keep seg_idx/progress/forward current: the lane guard (wrong-side
        # check) and API state read them, but FREE mode never updates them
        # on its own - without this, every check silently measures against
        # the segment the car spawned on. Cheap per frame (one projection);
        # the full nearest-segment search only runs when the car has left
        # the current segment (junctions, off-road excursions).
        if network is not None:
            self._refresh_segment_if_left(network)
    
    def _refresh_segment_if_left(self, network):
        """Re-derive seg_idx/progress/forward once the car has left its
        current segment (FREE mode only - BICYCLE mode tracks this itself)."""
        seg = network.segments[self.seg_idx]
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-9:
            self.snap_to_road(network)
            return
        t = ((self.x - seg.x1) * dx + (self.y - seg.y1) * dy) / length_sq
        proj_x = seg.x1 + t * dx
        proj_y = seg.y1 + t * dy
        lat_px = math.hypot(self.x - proj_x, self.y - proj_y)
        half_width_px = (seg.width / 2.0 + 1.0) * config.PIXELS_PER_METER
        if not (-0.15 <= t <= 1.15) or lat_px > half_width_px:
            self.snap_to_road(network)
    
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

    # Interpolated render position (x, y, heading), set by the main loop
    # every frame. Physics runs in fixed 1/60 s substeps; a rendered frame
    # can contain 0 or 2 of them (the accumulator aliasing against the
    # render rate), which without interpolation makes the car freeze for a
    # frame and then jump - visible as a periodic 2-3 px hop. Rendering at
    # lerp(prev_state, curr_state, alpha) instead moves smoothly every
    # frame. None = draw the live state (frozen / before first step).
    _render_state: tuple[float, float, float] | None = None

    def render_body_center(self) -> tuple[float, float]:
        """body_center() at the interpolated render position.

        Only for DRAWING. Physics and validation keep using the exact state
        via body_center().
        """
        if self._render_state is None:
            return self.body_center()
        x, y, h = self._render_state
        rad = math.radians(h)
        off = config.REAR_AXLE_OFFSET_M * config.PIXELS_PER_METER
        return x + math.sin(rad) * off, y + math.cos(rad) * off

    def render_heading(self) -> float:
        """Heading at the interpolated render position (drawing only)."""
        if self._render_state is None:
            return self.heading
        return self._render_state[2]

    def is_on_road(self, network) -> bool:
        """Check if the car (all four corners, not just its center) is on
        any road.

        Delegates to RoadNetwork.is_car_on_road(), which tests the car's
        four bounding-box corners against the exact same paved-area polygon
        that gets rendered (rounded bends and junction fillets included).
        """
        bx, by = self.body_center()
        return network.is_car_on_road(bx, by, self.heading)
