# Car Class - Pure Physics and State
# No input handling - controlled by Driver classes

from __future__ import annotations

import math
import pygame

from . import config
from .turning_system import TurningSystem, TurnPlan


class Car:
    """A car with physics, state, and rendering. Controlled by a Driver."""
    
    # Class variable: cached sprite images
    _sprite_cache = {}
    
    def __init__(self, x: float, y: float, heading: float, seg_idx: int, driver=None):
        """x, y in world pixels. heading in degrees (0 = up/north)."""
        # Position and orientation
        self.x = x
        self.y = y
        self.heading = heading
        
        # Speed
        self.speed = 0.0  # m/s
        self.target_speed = 0.0
        
        # Road following state (for RAILS mode)
        self.seg_idx = seg_idx
        self.progress = 0.5  # 0.0 to 1.0 along segment
        self.forward = True  # Direction on segment
        
        # Driver controlling this car
        self.driver = driver
        
        # Turning system
        self.turning_system = TurningSystem(max_lateral_accel=5.0)
        self.active_turn: TurnPlan | None = None
        self._cached_turn_node: str | None = None
        self._cached_turn_to_seg: int | None = None
        self._cached_turn_plan: TurnPlan | None = None
        self._warned_no_arc_for_node: str | None = None
        
        # Visual state
        self._braking = False
        self._accelerating = False
        
        # Smooth heading transition
        self._heading_transition = False
        self._heading_transition_progress = 0.0
        self._heading_start = 0.0
        self._heading_target = 0.0
        self._heading_transition_duration = 0.3
        
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
            blinker_left: bool (RAILS mode, from driver state)
            blinker_right: bool (RAILS mode, from driver state)
        """
        old_x, old_y = self.x, self.y
        
        # Update visual state
        self._braking = control_input.get('brake', False)
        self._accelerating = control_input.get('accelerate', False)
        
        # Choose update mode based on driver type
        if self.driver and self.driver.get_name() == "RAILS":
            self._update_rails_mode(dt, network, control_input)
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
    
    # --- RAILS Mode Physics ---
    
    def _update_rails_mode(self, dt: float, network, control_input: dict):
        """Automatic road following mode with physics-based turning."""
        accel = control_input.get('accelerate', False)
        brake = control_input.get('brake', False)
        
        # Speed control
        if accel:
            self.target_speed = min(self.target_speed + config.CAR_ACCELERATION * dt, config.CAR_SPEED)
        elif brake:
            self.target_speed = max(self.target_speed - config.CAR_BRAKING * dt, 0)
            # Direct braking
            self.speed -= config.CAR_BRAKING * dt
        
        # Approach target speed
        if self.speed < self.target_speed:
            self.speed += config.CAR_ACCELERATION * dt
        elif self.speed > self.target_speed:
            self.speed -= config.CAR_BRAKING * dt * 0.3
        
        self.speed = max(0, min(config.CAR_SPEED, self.speed))
        
        # Move along road
        if self.speed > 0:
            # Check if we're currently executing a turn
            if self.active_turn:
                self._execute_active_turn(dt, network)
            else:
                # Normal segment following
                self._update_position_rails(dt, network)
    
    def _execute_active_turn(self, dt: float, network):
        """Execute current turn using circular arc."""
        distance_m = self.speed * dt
        
        # Move along arc
        x, y, heading, new_progress = self.turning_system.execute_turn(
            self.active_turn, distance_m
        )
        
        self.x = x
        self.y = y
        self.heading = heading
        self.active_turn.progress = new_progress
        
        # Check if turn complete
        if new_progress >= 1.0:
            # Transition to next segment
            self.seg_idx = self.active_turn.to_seg_idx
            to_seg = network.segments[self.seg_idx]
            
            # Set direction on new segment
            self.forward = (to_seg.start_node == self.active_turn.junction_node)
            self.progress = 0.0 if self.forward else 1.0
            
            # IMPORTANT: The arc's own geometry (circle math) does not
            # necessarily land exactly on the network's real junction node
            # coordinates. Snap position to the actual node here so the
            # next frame's segment-following code (which recomputes x/y
            # directly from network.segments[...].x1/y1) doesn't see a
            # sudden jump (this was causing teleportation-watchdog
            # violations immediately after a completed arc turn).
            node_xy = network.nodes.get(self.active_turn.junction_node)
            if node_xy is not None:
                self.x, self.y = node_xy
            
            # Notify driver that turn completed
            if self.driver and hasattr(self.driver, 'clear_blinker_if_turned'):
                self.driver.clear_blinker_if_turned(
                    self, network, 
                    self.active_turn.from_seg_idx, 
                    self.active_turn.to_seg_idx
                )
            
            # Clear active turn
            self.active_turn = None
    
    def _update_position_rails(self, dt: float, network):
        """Move along current segment, checking for upcoming turns."""
        if self.seg_idx >= len(network.segments):
            return
        
        seg = network.segments[self.seg_idx]
        distance_m = self.speed * dt
        distance_frac = distance_m / seg.length if seg.length > 0 else 0
        
        # Check if approaching junction and should plan/execute turn
        approaching_junction = False
        junction_node = None
        
        if self.forward and self.progress > 0.7:
            approaching_junction = True
            junction_node = seg.end_node
        elif not self.forward and self.progress < 0.3:
            approaching_junction = True
            junction_node = seg.start_node
        
        # Re-check every frame while approaching (not just once!) so we can
        # keep braking harder as long as the turn is not yet feasible at the
        # current speed. The expensive arc search itself is cached per node
        # inside _check_and_plan_turn, so this is cheap to call repeatedly.
        if approaching_junction and junction_node and not self.active_turn:
            self._check_and_plan_turn(network, junction_node, dt)
        
        # Clear turn-plan cache once we're far from any junction again
        if hasattr(self, '_cached_turn_node'):
            if (self.forward and self.progress < 0.5) or (not self.forward and self.progress > 0.5):
                self._cached_turn_node = None
                self._cached_turn_plan = None
        
        # Move along segment
        if self.forward:
            self.progress += distance_frac
        else:
            self.progress -= distance_frac
        
        # Check for segment end (only if NOT in an active turn)
        # If we have an active turn, the arc execution handles the transition
        if not self.active_turn:
            if self.progress >= 1.0:
                self._handle_segment_end(network, seg.end_node)
            elif self.progress <= 0.0:
                self._handle_segment_end(network, seg.start_node)
        
        # Update position on segment
        seg = network.segments[self.seg_idx]
        t = max(0.0, min(1.0, self.progress))
        self.x = seg.x1 + t * (seg.x2 - seg.x1)
        self.y = seg.y1 + t * (seg.y2 - seg.y1)
        
        # Lane offset (right side)
        pppm = config.PIXELS_PER_METER
        lane_offset = (seg.width / 4) * pppm if not seg.oneway else 0
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_len = math.hypot(dx, dy)
        if seg_len > 0:
            nx = -dy / seg_len
            ny = dx / seg_len
            if self.forward:
                self.x -= nx * lane_offset
                self.y -= ny * lane_offset
            else:
                self.x += nx * lane_offset
                self.y += ny * lane_offset
        
        # Heading (smooth from segment direction, will be overridden during turn)
        if self.forward:
            self.heading = math.degrees(math.atan2(dx, dy))
        else:
            self.heading = math.degrees(math.atan2(-dx, -dy))
    
    def _check_and_plan_turn(self, network, junction_node: str, dt: float):
        """Check if we should plan/execute a turn at upcoming junction.

        Called every frame while approaching a junction (until the turn
        actually starts). Re-evaluates feasibility at the CURRENT speed
        every time, so if the turn isn't safe yet we brake hard and try
        again next frame — we never start an arc that doesn't fit, and we
        never blindly attempt a 90° turn at highway speed.
        """
        # Get turn intention from driver
        if self.driver and hasattr(self.driver, 'pending_turn'):
            turn = self.driver.pending_turn or "straight"
        else:
            turn = "straight"
        
        # Get next segment
        next_seg_idx = network.choose_next_segment(self.seg_idx, junction_node, turn)
        
        if next_seg_idx is None or next_seg_idx == self.seg_idx:
            return  # No valid turn or dead end
        
        # Recompute fresh every frame using the car's CURRENT position and
        # heading as the arc's anchor (validate_arc_on_road only checks 2
        # segments so this is cheap). We deliberately don't cache the plan
        # across frames: while braking towards feasibility the car keeps
        # moving, and an arc anchored to a stale position would itself
        # reintroduce a small jump the moment the turn actually starts.
        turn_plan = self.turning_system.plan_turn(
            self.x, self.y, self.speed, self.heading,
            self.seg_idx, next_seg_idx, junction_node,
            network
        )
        self._cached_turn_node = junction_node
        self._cached_turn_to_seg = next_seg_idx
        self._cached_turn_plan = turn_plan
        
        if not turn_plan:
            # No arc fits within road boundaries at any radius we tried —
            # almost always because we're going too fast for this turn given
            # the available road length. Brake hard and retry next frame:
            # as speed drops, the required radius shrinks and a fitting arc
            # may become possible. If we run out of road before that
            # happens, the segment-end fallback (instant transition) kicks
            # in instead of attempting a geometrically invalid arc.
            if not getattr(self, '_warned_no_arc_for_node', None) == junction_node:
                self._warned_no_arc_for_node = junction_node
                print(f"⚠️ Cannot plan turn {self.seg_idx} → {next_seg_idx} at {self.speed * 3.6:.0f} km/h: "
                      f"no arc fits within road boundaries — braking hard")
            self.speed = max(0.0, self.speed - config.CAR_BRAKING * dt)
            self.target_speed = min(self.target_speed, self.speed)
            self._braking = True
            return
        
        # Check feasibility AT CURRENT SPEED (re-evaluated every frame)
        seg = network.segments[self.seg_idx]
        feasibility = self.turning_system.check_turn_feasible(
            self.speed, self.progress, seg.length, turn_plan, self.forward
        )
        
        if not feasibility['feasible']:
            # Too fast for this turn given remaining distance to the
            # junction — brake hard (full braking force) and re-check next
            # frame. If we run out of road before becoming feasible, the
            # segment-end fallback (instant transition) kicks in instead of
            # us blindly attempting an oversized, off-geometry arc.
            self.speed = max(0.0, self.speed - config.CAR_BRAKING * dt)
            self.target_speed = min(self.target_speed, self.speed)
            self._braking = True
            return
        
        # Feasible now (possibly after braking on previous frames) - start it
        self.active_turn = turn_plan
        print(f"🔄 Starting turn: {self.seg_idx} → {next_seg_idx} at progress {self.progress:.2f} "
              f"(speed {self.speed * 3.6:.0f} km/h, radius {turn_plan.radius:.1f}m)")
    
    def _handle_segment_end(self, network, node_id: str):
        """Handle reaching end of current segment."""
        # Ask driver for turn decision
        if self.driver and hasattr(self.driver, 'pending_turn'):
            turn = self.driver.pending_turn or "straight"
        else:
            turn = "straight"
        
        next_seg = network.choose_next_segment(self.seg_idx, node_id, turn)
        
        if next_seg is not None and next_seg != self.seg_idx:
            old_seg = self.seg_idx
            self.seg_idx = next_seg
            new_seg = network.segments[next_seg]
            self.forward = (new_seg.start_node == node_id)
            self.progress = 0.0 if self.forward else 1.0
            
            # Notify driver that we turned
            if self.driver and hasattr(self.driver, 'clear_blinker_if_turned'):
                self.driver.clear_blinker_if_turned(self, network, old_seg, next_seg)
        else:
            # Dead end - turn around
            self.heading = (self.heading + 180) % 360
            self.forward = not self.forward
            self.progress = 0.9 if self.forward else 0.1
            self.speed = 0
            if self.driver and hasattr(self.driver, 'blinker_left'):
                self.driver.blinker_left = False
                self.driver.blinker_right = False
                self.driver.pending_turn = None
    
    def _update_heading_transition(self, dt: float):
        """Smooth heading interpolation."""
        self._heading_transition_progress += dt / self._heading_transition_duration
        
        if self._heading_transition_progress >= 1.0:
            self._heading_transition = False
            self.heading = self._heading_target
        else:
            t = self._heading_transition_progress
            t = t * t * (3 - 2 * t)  # smoothstep
            
            diff = self._heading_target - self._heading_start
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            self.heading = (self._heading_start + t * diff) % 360
    
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
        self.active_turn = None
        self.trail.clear()

    def is_on_road(self, network) -> bool:
        """Check if car is currently on any road."""
        pppm = config.PIXELS_PER_METER
        for seg in network.segments:
            half_width = (seg.width / 2) * pppm
            # Simple point-to-segment distance check
            dx = seg.x2 - seg.x1
            dy = seg.y2 - seg.y1
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                continue
            t = max(0, min(1, ((self.x - seg.x1) * dx + (self.y - seg.y1) * dy) / length_sq))
            proj_x = seg.x1 + t * dx
            proj_y = seg.y1 + t * dy
            dist = math.hypot(self.x - proj_x, self.y - proj_y)
            if dist <= half_width:
                return True
        return False
    
    # --- Rendering ---
    
    def draw(self, surface: pygame.Surface, camera):
        """Draw the car sprite."""
        sx, sy = camera.world_to_screen(self.x, self.y)
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
        if self.driver and self.driver.get_name() == "RAILS":
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
    
    def _draw_headlights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        for sign in (-1, 1):
            hx = sx + math.sin(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            hx += math.cos(rad) * sign * half_wid * 0.6
            hy = sy - math.cos(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            hy += math.sin(rad) * sign * half_wid * 0.6
            pygame.draw.circle(surface, (255, 255, 220), (int(hx), int(hy)), 2)
    
    def _draw_taillights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        for sign in (-1, 1):
            tx = sx - math.sin(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            tx += math.cos(rad) * sign * half_wid * 0.6
            ty = sy + math.cos(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            ty += math.sin(rad) * sign * half_wid * 0.6
            color = (255, 40, 0) if self._braking else (150, 0, 0)
            pygame.draw.circle(surface, color, (int(tx), int(ty)), 2)
    
    def _draw_blinkers(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        """Draw blinker lights (orange, flashing) in RAILS mode."""
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
