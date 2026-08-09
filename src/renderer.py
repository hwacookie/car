# Renderer
# Draws the road network, minimap and HUD onto the Pygame surface.

from __future__ import annotations

import math
import pygame

from . import config
from .road_network import RoadNetwork
from .camera import Camera


class Renderer:
    def __init__(self, network: RoadNetwork, camera: Camera):
        self.network = network
        self.camera = camera
        self._font: pygame.font.Font | None = None
        try:
            pygame.font.init()
            self._font = pygame.font.SysFont("monospace", 14, bold=True)
        except Exception:
            self._font = None  # fonts unavailable (e.g. headless); skip text

    # --- Full scene ---

    def draw(self, surface: pygame.Surface, car_speed: float):
        """Draw the full game scene."""
        self.draw_roads(surface)
        self.draw_minimap(surface, car_speed)

    # --- Roads ---

    def draw_roads(self, surface: pygame.Surface):
        """Draw all road segments in world coordinates, transformed via camera."""
        w, h = surface.get_size()
        for seg in self.network.segments:
            color = config.ROAD_TYPES.get(seg.highway, {}).get("color", (150, 150, 150))
            width = seg.width * config.PIXELS_PER_METER * self.camera.zoom
            width = max(1, int(width))

            sx1, sy1 = self.camera.world_to_screen(seg.x1, seg.y1)
            sx2, sy2 = self.camera.world_to_screen(seg.x2, seg.y2)

            # Cull segments completely off-screen
            margin = width + 2
            if (sx1 < -margin and sx2 < -margin or
                sx1 > w + margin and sx2 > w + margin or
                sy1 < -margin and sy2 < -margin or
                sy1 > h + margin and sy2 > h + margin):
                continue

            pygame.draw.line(surface, color, (int(sx1), int(sy1)), (int(sx2), int(sy2)), width)

    # --- Minimap ---

    def draw_minimap(self, surface: pygame.Surface, car_speed: float):
        """Draw minimap in top-right corner with roads + car dot."""
        mm_w = config.MINIMAP_SIZE
        mm_h = config.MINIMAP_SIZE
        mm_x = surface.get_width() - mm_w - config.MINIMAP_MARGIN
        mm_y = config.MINIMAP_MARGIN

        # Background
        mm_rect = pygame.Rect(mm_x, mm_y, mm_w, mm_h)
        pygame.draw.rect(surface, config.MINIMAP_BG, mm_rect)
        pygame.draw.rect(surface, config.MINIMAP_BORDER, mm_rect, 2)

        # Scale world → minimap
        bounds = self.network.bounds
        sx = mm_w / bounds[2]
        sy = mm_h / bounds[3]

        # Roads
        for seg in self.network.segments:
            color = config.ROAD_TYPES.get(seg.highway, {}).get("color", (150, 150, 150))
            x1 = mm_x + seg.x1 * sx
            y1 = mm_y + seg.y1 * sy
            x2 = mm_x + seg.x2 * sx
            y2 = mm_y + seg.y2 * sy
            pygame.draw.line(surface, color, (x1, y1), (x2, y2), 1)

        # Car dot
        car_x, car_y = self.camera.x, self.camera.y
        cx = mm_x + car_x * sx
        cy = mm_y + car_y * sy
        pygame.draw.circle(surface, config.MINIMAP_CAR_COLOR, (int(cx), int(cy)), 3)

        # Speed text
        if self._font is not None:
            speed_kmh = int(car_speed * 3.6)
            txt = self._font.render(f"{speed_kmh} km/h", True, (255, 255, 255))
            surface.blit(txt, (mm_x + 6, mm_y + mm_h - 22))
