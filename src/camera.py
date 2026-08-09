# Camera / Viewport
# Follows the car with smooth interpolation and supports zoom in/out.

from __future__ import annotations

from . import config


class Camera:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.x = 0.0       # world pixel centre
        self.y = 0.0
        self.zoom = 1.0
        self.lerp_factor = 0.08   # smooth follow speed

    def update(self, target_x: float, target_y: float, world_w: float, world_h: float):
        """Smoothly move camera toward target, clamped to world bounds."""
        self.x += (target_x - self.x) * self.lerp_factor
        self.y += (target_y - self.y) * self.lerp_factor

        # Clamp so camera never shows outside the world
        half_w = (self.width / 2) / self.zoom
        half_h = (self.height / 2) / self.zoom
        self.x = max(half_w, min(world_w - half_w, self.x))
        self.y = max(half_h, min(world_h - half_h, self.y))

    def world_to_screen(self, wx: float, wy: float) -> tuple[float, float]:
        sx = (wx - self.x) * self.zoom + self.width / 2
        sy = (self.y - wy) * self.zoom + self.height / 2   # north = up
        return sx, sy

    def zoom_in(self):
        self.zoom = min(self.zoom * config.ZOOM_STEP, config.MAX_ZOOM)

    def zoom_out(self):
        self.zoom = max(self.zoom / config.ZOOM_STEP, config.MIN_ZOOM)

    def handle_zoom(self, delta: float):
        """Mouse wheel delta (>0 = zoom in, <0 = zoom out)."""
        if delta > 0:
            self.zoom_in()
        elif delta < 0:
            self.zoom_out()
