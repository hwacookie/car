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

    def draw(self, surface: pygame.Surface, car):
        """Draw the full game scene."""
        self.draw_roads(surface)
        self.draw_minimap(surface, car)

    # --- Roads ---

    def draw_roads(self, surface: pygame.Surface):
        """Draw all road segments with casing + junction circles.

        Two passes: first a dark casing (outline) for every segment and node,
        then the road-coloured fill on top. This makes junctions look smooth
        and road-like instead of butted line ends.
        """
        w, h = surface.get_size()
        zoom = self.camera.zoom
        pppm = config.PIXELS_PER_METER

        # --- Pass 1: casing (dark outline) ---
        for seg in self.network.segments:
            width = max(1, int(seg.width * pppm * zoom)) + 2
            sx1, sy1 = self.camera.world_to_screen(seg.x1, seg.y1)
            sx2, sy2 = self.camera.world_to_screen(seg.x2, seg.y2)
            if self._offscreen(sx1, sy1, sx2, sy2, w, h, width):
                continue
            pygame.draw.line(surface, config.ROAD_EDGE_COLOR,
                             (int(sx1), int(sy1)), (int(sx2), int(sy2)), width)

        # Junction circles (casing colour)
        for nid, (half, _hw) in self.network.node_max_width.items():
            nx, ny = self.network.nodes[nid]
            sx, sy = self.camera.world_to_screen(nx, ny)
            r = int(half * zoom) + 1
            if sx < -r or sx > w + r or sy < -r or sy > h + r:
                continue
            pygame.draw.circle(surface, config.ROAD_EDGE_COLOR, (int(sx), int(sy)), r)

        # --- Pass 2: road fill ---
        for seg in self.network.segments:
            color = config.ROAD_TYPES.get(seg.highway, {}).get("color", (150, 150, 150))
            width = max(1, int(seg.width * pppm * zoom))
            sx1, sy1 = self.camera.world_to_screen(seg.x1, seg.y1)
            sx2, sy2 = self.camera.world_to_screen(seg.x2, seg.y2)
            if self._offscreen(sx1, sy1, sx2, sy2, w, h, width):
                continue
            pygame.draw.line(surface, color,
                             (int(sx1), int(sy1)), (int(sx2), int(sy2)), width)

        # Junction circles (fill colour of widest road at node)
        for nid, (half, highway) in self.network.node_max_width.items():
            nx, ny = self.network.nodes[nid]
            sx, sy = self.camera.world_to_screen(nx, ny)
            r = int(half * zoom)
            if r < 1 or sx < -r or sx > w + r or sy < -r or sy > h + r:
                continue
            color = config.ROAD_TYPES.get(highway, {}).get("color", (170, 170, 170))
            pygame.draw.circle(surface, color, (int(sx), int(sy)), r)

    @staticmethod
    def _offscreen(x1, y1, x2, y2, w, h, margin) -> bool:
        return (x1 < -margin and x2 < -margin or
                x1 > w + margin and x2 > w + margin or
                y1 < -margin and y2 < -margin or
                y1 > h + margin and y2 > h + margin)

    # --- Minimap ---

    def draw_minimap(self, surface: pygame.Surface, car):
        """Draw minimap in top-right corner with roads + car dot."""
        mm_w = config.MINIMAP_SIZE
        mm_h = config.MINIMAP_SIZE
        mm_x = surface.get_width() - mm_w - config.MINIMAP_MARGIN
        mm_y = config.MINIMAP_MARGIN

        # Background
        mm_rect = pygame.Rect(mm_x, mm_y, mm_w, mm_h)
        pygame.draw.rect(surface, config.MINIMAP_BG, mm_rect)
        pygame.draw.rect(surface, config.MINIMAP_BORDER, mm_rect, 2)

        # Scale world -> minimap
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
        cx = mm_x + car.x * sx
        cy = mm_y + car.y * sy
        pygame.draw.circle(surface, config.MINIMAP_CAR_COLOR, (int(cx), int(cy)), 3)

        # Speed text
        if self._font is not None:
            speed_kmh = int(car.speed * 3.6)
            txt = self._font.render(f"{speed_kmh} km/h", True, (255, 255, 255))
            surface.blit(txt, (mm_x + 6, mm_y + mm_h - 22))
