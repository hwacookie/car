# Renderer
# Draws the road network, minimap and HUD onto the Pygame surface.

from __future__ import annotations

import pygame

from . import config
from .road_network import RoadNetwork
from .camera import Camera
from .road_surface import RoadSurface, build_road_surface


class Renderer:
    def __init__(self, network: RoadNetwork, camera: Camera):
        self.network = network
        self.camera = camera
        self.road_surface = build_road_surface(network)
        self._font: pygame.font.Font | None = None
        try:
            pygame.font.init()
            self._font = pygame.font.SysFont("monospace", 14, bold=True)
        except Exception:
            self._font = None

    def draw(self, surface: pygame.Surface, car):
        self.road_surface.draw_to_screen(surface, self.camera)
        self.draw_minimap(surface, car)

    # --- Minimap ---
    def draw_minimap(self, surface: pygame.Surface, car):
        mm_w = config.MINIMAP_SIZE
        mm_h = config.MINIMAP_SIZE
        mm_x = surface.get_width() - mm_w - config.MINIMAP_MARGIN
        mm_y = config.MINIMAP_MARGIN

        mm_rect = pygame.Rect(mm_x, mm_y, mm_w, mm_h)
        pygame.draw.rect(surface, config.MINIMAP_BG, mm_rect)
        pygame.draw.rect(surface, config.MINIMAP_BORDER, mm_rect, 2)

        bounds = self.network.bounds
        sx = mm_w / bounds[2]
        sy = mm_h / bounds[3]

        for seg in self.network.segments:
            color = config.ROAD_TYPES.get(seg.highway, {}).get("color", (150, 150, 150))
            x1 = mm_x + seg.x1 * sx
            y1 = mm_y + seg.y1 * sy
            x2 = mm_x + seg.x2 * sx
            y2 = mm_y + seg.y2 * sy
            pygame.draw.line(surface, color, (x1, y1), (x2, y2), 1)

        cx = mm_x + car.x * sx
        cy = mm_y + car.y * sy
        pygame.draw.circle(surface, config.MINIMAP_CAR_COLOR, (int(cx), int(cy)), 3)

        if self._font is not None:
            speed_kmh = int(car.speed * 3.6)
            txt = self._font.render(f"{speed_kmh} km/h", True, (255, 255, 255))
            surface.blit(txt, (mm_x + 6, mm_y + mm_h - 22))