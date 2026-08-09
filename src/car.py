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
        self._was_braking = False

    # --- Input & Physics ---

    def handle_input(self, keys: dict, dt: float):
        """Process keyboard input and update physics.

        keys: pygame.key.ScancodeWrapper from pygame.key.get_pressed().
        dt: delta time in seconds.
        """
        accel = keys[pygame.K_UP] or keys[pygame.K_w]
        brake = keys[pygame.K_DOWN] or keys[pygame.K_s]
        left  = keys[pygame.K_LEFT] or keys[pygame.K_a]
        right = keys[pygame.K_RIGHT] or keys[pygame.K_d]

        prev_speed = self.speed

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

        # Track braking state (speed dropping while pressing brake)
        self._was_braking = brake and self.speed < prev_speed

        # Turning (only when moving)
        if self.speed > 0:
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
        """Draw the car sprite centred on screen (camera follows car)."""
        sx = surface.get_width() // 2
        sy = surface.get_height() // 2
        scale = zoom

        half_len = (config.CAR_LENGTH / 2) * config.PIXELS_PER_METER * scale
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale

        # Build an unrotated car surface, then rotate it
        src = pygame.Surface((half_wid * 2 + 2, half_len * 2 + 2), pygame.SRCALPHA)
        # Body
        pygame.draw.rect(src, (180, 30, 30),
                         pygame.Rect(1, 1, half_wid * 2, half_len * 2))
        # Front strip (lighter, at the top = front)
        pygame.draw.rect(src, (215, 60, 60),
                         pygame.Rect(1 + half_wid * 0.3, 1, half_wid * 1.4, half_len * 0.35))

        rotated = pygame.transform.rotozoom(src, -self.heading, 1)
        surface.blit(rotated, (sx - rotated.get_width() // 2,
                               sy - rotated.get_height() // 2))

        # Lights
        self._draw_headlights(surface, sx, sy, scale)
        self._draw_taillights(surface, sx, sy, scale)

    def _draw_headlights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        """Draw two headlight circles at the car's front."""
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        for sign in (-1, 1):
            hx = (sx + math.sin(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
                  + math.cos(rad) * sign * half_wid * 0.6)
            hy = (sy - math.cos(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
                  + math.sin(rad) * sign * half_wid * 0.6)
            r = max(2, int(4 * scale))
            pygame.draw.circle(surface, (255, 255, 200), (int(hx), int(hy)), r)

    def _draw_taillights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        """Draw two taillight circles at the car's rear — bright red when braking."""
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        for sign in (-1, 1):
            tx = (sx - math.sin(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
                  + math.cos(rad) * sign * half_wid * 0.6)
            ty = (sy + math.cos(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
                  + math.sin(rad) * sign * half_wid * 0.6)
            r = max(2, int(4 * scale))
            color = (255, 30, 0) if self._was_braking else (120, 0, 0)
            pygame.draw.circle(surface, color, (int(tx), int(ty)), r)
