# Car Entity
# Two driving modes:
#   1. FREE mode: manual steering (W/S = accel/brake, A/D = steer)
#   2. RAILS mode: follow road automatically (W/S = accel/brake, A/D = blinker/turn at junctions)
# Press TAB to toggle between modes.

from __future__ import annotations

import math
import pygame

from . import config


class Car:
    def __init__(self, x: float, y: float, heading: float, seg_idx: int):
        """x, y in world pixels. heading in degrees (0 = up/north). seg_idx is current segment."""
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = 0.0  # m/s
        self.target_speed = 0.0  # for automatic speed control in rails mode

        # Teleportation watchdog
        self._last_x = x
        self._last_y = y
        self._frames_to_skip = 5  # Skip watchdog for first few frames (initial positioning)

        # Driving mode: "free" or "rails"
        self.mode = "rails"  # Start in rails mode (automatic road following)

        # Rails mode state
        self.seg_idx = seg_idx
        self.progress = 0.5
        self.forward = True
        self.blinker_left = False
        self.blinker_right = False
        self._blinker_timer = 0.0
        self._pending_turn = None  # "left", "right", or None

        # Junction transition (for smooth turning)
        self._in_junction = False
        self._junction_from_seg = None
        self._junction_to_seg = None
        self._junction_progress = 0.0
        self._junction_duration = 1.0  # will be recalculated per junction
        self._junction_start_pos = (0.0, 0.0)
        self._junction_end_pos = (0.0, 0.0)
        self._junction_start_heading = 0.0
        self._junction_end_heading = 0.0

        # Visual state
        self._braking = False
        self._accelerating = False
        self._last_left = False
        self._last_right = False
        self._last_tab = False

    # --- Input & Physics ---

    def handle_input(self, keys: dict, dt: float, network):
        """Process keyboard input and update physics."""
        # Save old position for teleportation check
        old_x, old_y = self.x, self.y
        
        # Toggle mode with TAB
        if keys[pygame.K_TAB] and not self._last_tab:
            self.mode = "rails" if self.mode == "free" else "free"
            if self.mode == "rails":
                self._snap_to_road(network)
                self.target_speed = self.speed
            print(f"Driving mode: {self.mode.upper()}")
        self._last_tab = keys[pygame.K_TAB]

        if self.mode == "free":
            self._handle_free_mode(keys, dt)
        else:
            self._handle_rails_mode(keys, dt, network)
        
        # Watchdog: check for teleportation (skip first few frames for initial positioning)
        if self._frames_to_skip > 0:
            self._frames_to_skip -= 1
            return
        
        dx = self.x - old_x
        dy = self.y - old_y
        distance_moved = math.hypot(dx, dy)
        pppm = config.PIXELS_PER_METER
        distance_m = distance_moved / pppm
        
        # Maximum allowed: speed * dt * 2 + generous margin for segment transitions
        # Segment transitions can cause jumps due to lane offset differences
        max_allowed_m = max(50, self.speed * dt * 3 + 20)
        
        if distance_m > max_allowed_m:
            # Teleportation detected!
            import traceback
            error_msg = (
                f"\n{'='*70}\n"
                f"TELEPORTATION DETECTED!\n"
                f"{'='*70}\n"
                f"Old position: ({old_x:.1f}, {old_y:.1f})\n"
                f"New position: ({self.x:.1f}, {self.y:.1f})\n"
                f"Distance: {distance_m:.1f}m (max allowed: {max_allowed_m:.1f}m)\n"
                f"Speed: {self.speed:.1f} m/s ({self.speed * 3.6:.0f} km/h)\n"
                f"Mode: {self.mode}\n"
                f"Segment: {self.seg_idx}, Progress: {self.progress:.3f}, Forward: {self.forward}\n"
                f"dt: {dt:.4f}s\n"
                f"{'='*70}\n"
            )
            print(error_msg)
            traceback.print_stack()
            # Revert position to prevent jump
            self.x = old_x
            self.y = old_y
            self.speed = 0
            raise RuntimeError(error_msg)

    def _handle_free_mode(self, keys: dict, dt: float):
        """Free driving: manual steering, cruise control."""
        accel = keys[pygame.K_UP] or keys[pygame.K_w]
        brake = keys[pygame.K_DOWN] or keys[pygame.K_s]
        left = keys[pygame.K_LEFT] or keys[pygame.K_a]
        right = keys[pygame.K_RIGHT] or keys[pygame.K_d]

        self._braking = brake
        self._accelerating = accel

        if accel:
            self.speed += config.CAR_ACCELERATION * dt
        elif brake:
            self.speed -= config.CAR_BRAKING * dt

        self.speed = max(0, min(config.CAR_SPEED, self.speed))
        self.target_speed = self.speed

        if self.speed > 0:
            turn_factor = max(0.3, 1.0 - self.speed / config.CAR_SPEED * 0.7)
            turn_rate = config.CAR_TURN_SPEED * turn_factor * dt
            if left:
                self.heading -= turn_rate
            if right:
                self.heading += turn_rate
            self.heading = self.heading % 360

        rad = math.radians(self.heading)
        dx = math.sin(rad) * self.speed * dt * config.PIXELS_PER_METER
        dy = math.cos(rad) * self.speed * dt * config.PIXELS_PER_METER
        self.x += dx
        self.y += dy

    def _handle_rails_mode(self, keys: dict, dt: float, network):
        """Rails mode: follow road, blinkers for turning."""
        accel = keys[pygame.K_UP] or keys[pygame.K_w]
        brake = keys[pygame.K_DOWN] or keys[pygame.K_s]
        left = keys[pygame.K_LEFT] or keys[pygame.K_a]
        right = keys[pygame.K_RIGHT] or keys[pygame.K_d]

        self._braking = brake
        self._accelerating = accel

        # Turn signals (tap to toggle)
        if left and not self._last_left:
            self.blinker_left = not self.blinker_left
            if self.blinker_left:
                self.blinker_right = False
                self._pending_turn = "left"
            else:
                self._pending_turn = None
        if right and not self._last_right:
            self.blinker_right = not self.blinker_right
            if self.blinker_right:
                self.blinker_left = False
                self._pending_turn = "right"
            else:
                self._pending_turn = None
        self._last_left = left
        self._last_right = right

        self._blinker_timer += dt

        # Speed control
        if accel:
            self.target_speed = min(self.target_speed + config.CAR_ACCELERATION * dt, config.CAR_SPEED)
        elif brake:
            self.target_speed = max(self.target_speed - config.CAR_BRAKING * dt, 0)
            # DIRECT braking: also immediately reduce speed when S is pressed
            self.speed -= config.CAR_BRAKING * dt

        # Automatic braking before turns at real junctions (3+ connections)
        # Only brake if "rechts vor links" applies (road from the right)
        if self._pending_turn:
            seg = network.segments[self.seg_idx]
            node = seg.end_node if self.forward else seg.start_node
            
            # Only brake at real junctions (degree >= 3) with right-of-way conflict
            node_deg = network.node_degree.get(node, 2)
            if node_deg >= 3 and network.has_right_of_way_conflict(self.seg_idx, node):
                next_seg = network.choose_next_segment(self.seg_idx, node, self._pending_turn)
                if next_seg is not None and next_seg != self.seg_idx:
                    turn_angle = abs(network.get_exit_angle(self.seg_idx, next_seg))
                    # Determine safe turn speed based on angle
                    if turn_angle > 90:
                        safe_speed = 20 / 3.6  # 20 km/h for sharp turns
                    elif turn_angle > 60:
                        safe_speed = 30 / 3.6  # 30 km/h
                    elif turn_angle > 30:
                        safe_speed = 40 / 3.6  # 40 km/h
                    else:
                        safe_speed = self.target_speed  # gentle turn, no braking needed

                    # Calculate remaining distance to junction
                    if self.forward:
                        remaining_distance = seg.length * (1.0 - self.progress)
                    else:
                        remaining_distance = seg.length * self.progress
                    
                    # Calculate required braking distance: s = (v₁² - v₂²) / (2a)
                    if self.speed > safe_speed:
                        braking_distance = (self.speed**2 - safe_speed**2) / (2 * config.CAR_BRAKING)
                        
                        # Start braking when remaining distance <= braking distance + safety margin
                        safety_margin = 5.0  # 5 meters buffer
                        if remaining_distance <= braking_distance + safety_margin:
                            # Brake with FULL force (emergency braking for curves)
                            self.speed -= config.CAR_BRAKING * dt
                            self._braking = True
                        else:
                            # Not yet time to brake - maintain target speed
                            if self.speed < self.target_speed:
                                self.speed += config.CAR_ACCELERATION * dt
                            elif self.speed > self.target_speed:
                                self.speed -= config.CAR_BRAKING * dt * 0.3
                    else:
                        # Already at safe speed or below
                        if self.speed < self.target_speed:
                            self.speed += config.CAR_ACCELERATION * dt
                        elif self.speed > self.target_speed:
                            self.speed -= config.CAR_BRAKING * dt * 0.3
                else:
                    # No valid turn — just maintain speed
                    if self.speed < self.target_speed:
                        self.speed += config.CAR_ACCELERATION * dt
                    elif self.speed > self.target_speed:
                        self.speed -= config.CAR_BRAKING * dt * 0.3
            else:
                # Not a real junction or no right-of-way conflict
                # Don't brake, just maintain target speed
                if self.speed < self.target_speed:
                    self.speed += config.CAR_ACCELERATION * dt
                elif self.speed > self.target_speed:
                    self.speed -= config.CAR_BRAKING * dt * 0.3
        else:
            # No turn pending — accelerate to target speed
            if self.speed < self.target_speed:
                self.speed += config.CAR_ACCELERATION * dt
            elif self.speed > self.target_speed:
                self.speed -= config.CAR_BRAKING * dt * 0.3

        self.speed = max(0, min(config.CAR_SPEED, self.speed))

        # Move along the road
        if self.speed > 0:
            self._update_position_rails(dt, network)

    def _update_position_rails(self, dt: float, network):
        """Move along current segment."""
        if self.seg_idx >= len(network.segments):
            return

        seg = network.segments[self.seg_idx]
        distance_m = self.speed * dt
        distance_frac = distance_m / seg.length if seg.length > 0 else 0

        if self.forward:
            self.progress += distance_frac
        else:
            self.progress -= distance_frac

        # Check if we reached end of segment
        if self.progress >= 1.0:
            node = seg.end_node
            node_deg = network.node_degree.get(node, 2)
            turn = self._pending_turn or "straight"
            next_seg = network.choose_next_segment(self.seg_idx, node, turn)
            
            if next_seg is not None and next_seg != self.seg_idx:
                # Calculate turn angle to determine if we actually turned
                turn_angle = network.get_exit_angle(self.seg_idx, next_seg)
                
                # Switch to next segment
                old_seg_idx = self.seg_idx
                self.seg_idx = next_seg
                new_seg = network.segments[next_seg]
                self.forward = (new_seg.start_node == node)
                
                # Set progress to start/end of new segment (near the shared node)
                if self.forward:
                    self.progress = 0.0
                else:
                    self.progress = 1.0
                
                # Clear blinker ONLY if we actually turned in the blinked direction
                if self.blinker_left and turn_angle < -30:  # Actually turned left
                    self.blinker_left = False
                    self._pending_turn = None
                elif self.blinker_right and turn_angle > 30:  # Actually turned right
                    self.blinker_right = False
                    self._pending_turn = None
                # Otherwise: keep blinker on (geradeaus or wrong direction available)
            else:
                # Dead end - turn around 180° and stop
                self.heading = (self.heading + 180) % 360
                self.forward = not self.forward
                self.progress = 0.9
                self.speed = 0
                # Clear blinker at dead end
                self.blinker_left = False
                self.blinker_right = False
                self._pending_turn = None

        elif self.progress <= 0.0:
            node = seg.start_node
            turn = self._pending_turn or "straight"
            next_seg = network.choose_next_segment(self.seg_idx, node, turn)
            
            if next_seg is not None and next_seg != self.seg_idx:
                # Calculate turn angle
                turn_angle = network.get_exit_angle(self.seg_idx, next_seg)
                
                self.seg_idx = next_seg
                new_seg = network.segments[next_seg]
                
                if new_seg.start_node == node:
                    self.forward = True
                    self.progress = 0.0
                else:
                    self.forward = False
                    self.progress = 1.0
                
                # Clear blinker ONLY if we actually turned in the blinked direction
                if self.blinker_left and turn_angle < -30:  # Actually turned left
                    self.blinker_left = False
                    self._pending_turn = None
                elif self.blinker_right and turn_angle > 30:  # Actually turned right
                    self.blinker_right = False
                    self._pending_turn = None
            else:
                # Dead end - turn around 180° and stop
                self.heading = (self.heading + 180) % 360
                self.forward = not self.forward
                self.progress = 0.1
                self.speed = 0
                self.blinker_left = False
                self.blinker_right = False
                self._pending_turn = None

        # Update position on segment
        seg = network.segments[self.seg_idx]
        t = max(0.0, min(1.0, self.progress))
        self.x = seg.x1 + t * (seg.x2 - seg.x1)
        self.y = seg.y1 + t * (seg.y2 - seg.y1)

        # Offset to right lane
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

        # Heading
        if self.forward:
            self.heading = math.degrees(math.atan2(dx, dy))
        else:
            self.heading = math.degrees(math.atan2(-dx, -dy))

    def _start_junction(self, from_idx: int, to_idx: int, node_id: str, network, turn_angle: float):
        """Begin smooth junction transition."""
        self._in_junction = True
        self._junction_from_seg = from_idx
        self._junction_to_seg = to_idx
        self._junction_progress = 0.0

        from_seg = network.segments[from_idx]
        to_seg = network.segments[to_idx]
        pppm = config.PIXELS_PER_METER
        
        # Junction distance: shorter for sharper turns (5-8m)
        if turn_angle > 90:
            junction_dist_m = 5.0  # very tight turn
        else:
            junction_dist_m = 6.0  # sharp turn
        
        # Duration based on current speed (distance / speed)
        # But minimum 0.3s even at high speed, maximum 1.5s at low speed
        self._junction_duration = max(0.3, min(1.5, junction_dist_m / max(self.speed, 3.0)))

        # Start position: current position (already with lane offset)
        self._junction_start_pos = (self.x, self.y)
        self._junction_start_heading = self.heading

        # End position: small distance into the new segment (with proper lane offset)
        to_seg_len = to_seg.length
        t_end = min(0.3, junction_dist_m / max(to_seg_len, 1.0))
        
        if to_seg.start_node == node_id:
            # Going forward on new segment
            base_x = to_seg.x1 + (to_seg.x2 - to_seg.x1) * t_end
            base_y = to_seg.y1 + (to_seg.y2 - to_seg.y1) * t_end
            
            # Apply RIGHT lane offset
            lane_offset = (to_seg.width / 4) * pppm if not to_seg.oneway else 0
            dx = to_seg.x2 - to_seg.x1
            dy = to_seg.y2 - to_seg.y1
            seg_len = math.hypot(dx, dy)
            if seg_len > 0:
                nx = -dy / seg_len
                ny = dx / seg_len
                # RIGHT side = negative perpendicular when going forward
                base_x -= nx * lane_offset
                base_y -= ny * lane_offset
            
            self._junction_end_pos = (base_x, base_y)
            self._junction_end_heading = math.degrees(math.atan2(dx, dy))
            self.forward = True
        else:
            # Going backward on new segment
            base_x = to_seg.x2 + (to_seg.x1 - to_seg.x2) * t_end
            base_y = to_seg.y2 + (to_seg.y1 - to_seg.y2) * t_end
            
            # Apply RIGHT lane offset
            lane_offset = (to_seg.width / 4) * pppm if not to_seg.oneway else 0
            dx = to_seg.x1 - to_seg.x2
            dy = to_seg.y1 - to_seg.y2
            seg_len = math.hypot(dx, dy)
            if seg_len > 0:
                nx = -dy / seg_len
                ny = dx / seg_len
                # RIGHT side = positive perpendicular when going backward
                base_x += nx * lane_offset
                base_y += ny * lane_offset
            
            self._junction_end_pos = (base_x, base_y)
            self._junction_end_heading = math.degrees(math.atan2(dx, dy))
            self.forward = False

    def _update_junction(self, dt: float, network):
        """Smoothly interpolate through junction turn."""
        # Use speed-based duration calculated in _start_junction
        self._junction_progress += dt / self._junction_duration

        if self._junction_progress >= 1.0:
            # Finished junction
            self._in_junction = False
            self.seg_idx = self._junction_to_seg
            self.progress = min(0.3, 6.0 / max(network.segments[self._junction_to_seg].length, 1.0))
            if not self.forward:
                self.progress = 1.0 - self.progress
            self.x = self._junction_end_pos[0]
            self.y = self._junction_end_pos[1]
            self.heading = self._junction_end_heading
        else:
            # Smooth interpolation (ease-in-out)
            t = self._junction_progress
            t = t * t * (3 - 2 * t)  # smoothstep

            # Position: linear interpolation
            self.x = self._junction_start_pos[0] + t * (self._junction_end_pos[0] - self._junction_start_pos[0])
            self.y = self._junction_start_pos[1] + t * (self._junction_end_pos[1] - self._junction_start_pos[1])

            # Heading: interpolate angle (shortest path)
            h1 = self._junction_start_heading
            h2 = self._junction_end_heading
            diff = h2 - h1
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            self.heading = (h1 + t * diff) % 360

    def _snap_to_road(self, network):
        """Snap car to nearest road segment (for mode switching)."""
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

    # --- Visuals ---

    def draw(self, surface: pygame.Surface, camera):
        """Draw the car sprite."""
        sx, sy = camera.world_to_screen(self.x, self.y)
        sx, sy = int(sx), int(sy)
        scale = camera.zoom

        half_len = (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale

        src = pygame.Surface((half_wid * 2 + 2, half_len * 2 + 2), pygame.SRCALPHA)
        pygame.draw.rect(src, (180, 30, 30), pygame.Rect(1, 1, half_wid * 2, half_len * 2))
        pygame.draw.rect(src, (215, 60, 60), pygame.Rect(1 + half_wid * 0.3, 1, half_wid * 1.4, half_len * 0.35))

        rotated = pygame.transform.rotozoom(src, -self.heading, 1)
        surface.blit(rotated, (sx - rotated.get_width() // 2, sy - rotated.get_height() // 2))

        self._draw_headlights(surface, sx, sy, scale)
        self._draw_taillights(surface, sx, sy, scale)
        if self.mode == "rails":
            self._draw_blinkers(surface, sx, sy, scale)

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
        """Draw blinker lights (orange, flashing) in rails mode."""
        if not (self.blinker_left or self.blinker_right):
            return

        if (self._blinker_timer % 0.5) > 0.25:
            return

        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)

        for side in ((-1, self.blinker_left), (1, self.blinker_right)):
            sign, active = side
            if not active:
                continue
            bx = sx + math.sin(rad) * config.CAR_LENGTH / 3 * config.PIXELS_PER_METER * scale
            bx += math.cos(rad) * sign * half_wid * 0.8
            by = sy - math.cos(rad) * config.CAR_LENGTH / 3 * config.PIXELS_PER_METER * scale
            by += math.sin(rad) * sign * half_wid * 0.8
            pygame.draw.circle(surface, (255, 180, 0), (int(bx), int(by)), 3)
