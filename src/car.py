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

        keys: dict mapping pygame key constants to bool.
        dt: delta time in seconds.
        """
        accel = keys.get(pygame.K_UP, False) or keys.get(pygame.K_w, False)
        brake = keys.get(pygame.K_DOWN, False) or keys.get(pygame.K_s, False)
        left  = keys.get(pygame.K_LEFT, False) or keys.get(pygame.K_a, False)
        right = keys.get(pygame.K_RIGHT, False) or keys.get(pygame.K_d, False)

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

        # Car body (rotated rectangle)
        body = pygame.Rect(-half_wid, -half_len, half_wid * 2, half_len * 2)
        rotated = pygame.transform.rotozoom(body, -self.heading, 1)
        body_surface = pygame.Surface(rotated.size, pygame.SRCALPHA)
        body_surface.fill((180, 30, 30))  # red body

        # Front strip (slightly lighter)
        front_strip = pygame.Rect(-half_wid * 0.7, -half_len, half_wid * 1.4, half_len * 0.35)
        rotated_front = pygame.transform.rotozoom(front_strip, -self.heading, 1)
        front_surface = pygame.Surface(rotated_front.size, pygame.SRCALPHA)
        front_surface.fill((210, 60, 60))
        body_surface.blit(front_surface, (rotated_front.centerx - body.width // 2,
                                          rotated_front.centery - body.height // 2))

        surface.blit(body_surface, (sx - rotated.width // 2, sy - rotated.height // 2))

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
