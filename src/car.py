# Car Entity
# Top-down car with acceleration, braking, smooth turning,
# headlights and taillights (brake lights).

from __future__ import annotations

import math
import pygame

from . import config


class Car:
    def __init__(self, x: float, y: float, heading: float):
        """x, y in world pixels. heading in degrees (0 = up/north)."""
        self.x = x
        self.y = y
        self.heading = heading    # degrees
        self.speed = 0.0          # m/s

    # --- Input & Physics ---

    def handle_input(self, keys: dict, dt: float):
        """Process keyboard input and update physics.

        keys: dict mapping pygame key constants to bool.
        dt: delta time in seconds.
        """
        accel = keys.get(pygame.K_UP, False) or keys.get(pygame.K_w, False)
        brake = keys.get(pygame.K_DOWN, False) or keys.get(pygame.K_s, False)
        left  = keys.get(pygame.K_LEFT, False) or keys.get(pygame.K_a, False)
        right = keys.get(pygame.K_RIGHT, False) or keys.get(pygame.K_d, False)

        # Acceleration / braking
        if accel:
            self.speed += config.CAR_ACCELERATION * dt
        elif brake:
            self.speed -= config.CAR_BRAKING * dt
        else:
            # Friction
            self.speed *= max(0, 1 - 2.0 * dt)

        # Clamp speed
        self.speed = max(0, min(config.CAR_SPEED, self.speed))

        # Turning (only when moving)
        turn_rate = config.CAR_TURN_SPEED * dt
        if left:
            self.heading -= turn_rate
        if right:
            self.heading += turn_rate
        self.heading = self.heading % 360

        # Move forward
        rad = math.radians(self.heading)
        dx = math.sin(rad) * self.speed * dt * config.PIXELS_PER_METER
        dy = -math.cos(rad) * self.speed * dt * config.PIXELS_PER_METER
        self.x += dx
        self.y += dy

    # --- Visuals ---

    def draw(self, surface: pygame.Surface, zoom: float):
        """Draw the car sprite onto the screen surface (in screen coords)."""
        sx, sy = surface.get_width() // 2, surface.get_height() // 2  # car is always centred
        scale = zoom
        half_len = (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale

        # Car body (rotated rectangle)
        body = pygame.Rect(-half_wid, -half_len, half_wid * 2, half_len * 2)

        # Front indicator (thin rectangle at front)
        front = pygame.Rect(-half_wid, -half_len, half_wid * 2, half_len * 0.3)

        rotated_body = pygame.transform.rotozoom(body, -self.heading, 1)
        rotated_front = pygame.transform.rotozoom(front, -self.heading, 1)

        body_surface = pygame.Surface(rotated_body.size, pygame.SRCALPHA)
        body_surface.fill((180, 30, 30))  # red car

        front_surface = pygame.Surface(rotated_front.size, pygame.SRCALPHA)
        front_surface.fill((210, 50, 50))  # slightly lighter front

        cx, cy = rotated_body.center
        body_surface.blit(front_surface, (rotated_front.centerx - cx, rotated_front.centery - cy), special_flags=pygame.BLEND_RGBA_ADD)

        surface.blit(body_surface, (sx - rotated_body.width // 2, sy - rotated_body.height // 2))

        # Headlights (white cones at front)
        if self.speed > 0:
            self._draw_headlights(surface, sx, sy, scale)

        # Taillights / brake lights
        self._draw_taillights(surface, sx, sy, scale)

    def _draw_headlights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        # Two small circles at front
        for offset in (-1, 1):
            hx = sx + math.sin(rad) * (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
            hy = sy - math.cos(rad) * (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
            hx += math.cos(rad) * offset * half_wid * 0.6
            hy += math.sin(rad) * offset * half_wid * 0.6
            r = max(2, 4 * scale)
            pygame.draw.circle(surface, (255, 255, 200), (int(hx), int(hy)), int(r))

    def _draw_taillights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        # Red circles at rear
        for offset in (-1, 1):
            tx = sx - math.sin(rad) * (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
            ty = sy + math.cos(rad) * (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
            tx += math.cos(rad) * offset * half_wid * 0.6
            ty += math.sin(rad) * offset * half_wid * 0.6
            r = max(2, 4 * scale)
            # Brighter red when braking (speed decreasing rapidly — we approximate)
            color = (255, 0, 0) if self.speed < config.CAR_SPEED * 0.5 else (150, 0, 0)
            pygame.draw.circle(surface, color, (int(tx), int(ty)), int(r))

    @property
    def is_braking(self) -> bool:
        return self.speed < config.CAR_SPEED * 0.3
