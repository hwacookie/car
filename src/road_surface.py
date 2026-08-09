# Road Surface Builder
# Builds road polygons from segments and pre-renders the entire network
# to an offscreen surface for fast, correct rendering.

from __future__ import annotations

import math
import pygame

from . import config
from .road_network import RoadNetwork, RoadSegment


class RoadSurface:
    """Pre-rendered road network surface."""

    def __init__(self, network: RoadNetwork):
        self.network = network
        self.surface: pygame.Surface | None = None
        self._scale = 1.0

    def build(self):
        """Render the entire road network to a surface."""
        world_w = self.network.world_width
        world_h = self.network.world_height

        max_size = 8192
        if world_w > max_size or world_h > max_size:
            self._scale = max_size / max(world_w, world_h)
        else:
            self._scale = 1.0

        surf_w = int(world_w * self._scale)
        surf_h = int(world_h * self._scale)

        self.surface = pygame.Surface((surf_w, surf_h), pygame.SRCALPHA)
        self.surface.fill((0, 0, 0, 0))

        pppm = config.PIXELS_PER_METER * self._scale

        for seg in self.network.segments:
            color = config.ROAD_TYPES.get(seg.highway, {}).get("color", (150, 150, 150))
            half_w = (seg.width / 2) * pppm

            # World -> surface coordinates (flip Y: world north=up, surface top=down)
            sx1 = seg.x1 * self._scale
            sy1 = surf_h - seg.y1 * self._scale
            sx2 = seg.x2 * self._scale
            sy2 = surf_h - seg.y2 * self._scale

            self._draw_road_segment(self.surface, sx1, sy1, sx2, sy2, half_w, color)

    def _draw_road_segment(self, surf: pygame.Surface, x1: float, y1: float,
                           x2: float, y2: float, half_w: float, color: tuple):
        """Draw a road segment as filled polygon with rounded ends."""
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < 1:
            return

        nx = -dy / length
        ny = dx / length

        r = max(1, int(half_w))

        # Rectangle
        x1a = x1 + nx * r
        y1a = y1 + ny * r
        x1b = x1 - nx * r
        y1b = y1 - ny * r
        x2a = x2 - nx * r
        y2a = y2 - ny * r
        x2b = x2 + nx * r
        y2b = y2 + ny * r

        pygame.draw.polygon(surf, color, [(x1a, y1a), (x1b, y1b), (x2a, y2a), (x2b, y2b)])
        pygame.draw.circle(surf, color, (int(x1), int(y1)), r)
        pygame.draw.circle(surf, color, (int(x2), int(y2)), r)

    def draw_to_screen(self, screen: pygame.Surface, camera):
        """Blit the visible portion of the road surface to the screen."""
        if self.surface is None:
            return

        surf_w = self.surface.get_width()
        surf_h = self.surface.get_height()
        world_w = self.network.world_width
        world_h = self.network.world_height

        # Visible world area (world coords, north=up)
        half_sw = (screen.get_width() / 2) / camera.zoom
        half_sh = (screen.get_height() / 2) / camera.zoom

        wx1 = camera.x - half_sw
        wy1 = camera.y - half_sh
        wx2 = camera.x + half_sw
        wy2 = camera.y + half_sh

        # Clamp to world bounds
        wx1 = max(0, min(world_w, wx1))
        wy1 = max(0, min(world_h, wy1))
        wx2 = max(0, min(world_w, wx2))
        wy2 = max(0, min(world_h, wy2))

        # Source rect on surface (surface Y=0 is top, world Y=0 is bottom)
        sx1 = int(wx1 * self._scale)
        # World Y=wy1 is at surface Y = surf_h - wy1*scale
        sy1 = int(surf_h - wy2 * self._scale)  # wy2 is the NORTH edge → top of visible area
        sx2 = int(wx2 * self._scale)
        sy2 = int(surf_h - wy1 * self._scale)  # wy1 is the SOUTH edge → bottom of visible area

        src_rect = pygame.Rect(sx1, sy1, sx2 - sx1, sy2 - sy1)

        # Destination rect on screen
        dst_w = int(src_rect.width * camera.zoom)
        dst_h = int(src_rect.height * camera.zoom)
        dst_rect = pygame.Rect(
            (screen.get_width() - dst_w) // 2,
            (screen.get_height() - dst_h) // 2,
            dst_w,
            dst_h,
        )

        if src_rect.width > 0 and src_rect.height > 0:
            sub = self.surface.subsurface(src_rect)
            scaled = pygame.transform.scale(sub, (dst_w, dst_h))
            screen.blit(scaled, dst_rect)


def build_road_surface(network: RoadNetwork) -> RoadSurface:
    """Factory function to build and return a RoadSurface."""
    rs = RoadSurface(network)
    rs.build()
    return rs