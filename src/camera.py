# Camera / Viewport
# The sim's camera: follows the car with smooth interpolation. Headless
# since M5 - there is no window; the remote renderer (Godot) mirrors
# x/y/zoom from /state, so this object only holds and updates the view.

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

        # Drag state
        self._dragging = False
        self._drag_start_mouse = (0, 0)
        self._drag_start_camera = (0.0, 0.0)

    def update(self, target_x: float, target_y: float, world_w: float, world_h: float,
               follow: bool = True):
        """Smoothly move camera toward target, clamped to world bounds.
        Only follows if not manually dragging and follow=True.
        """
        if not self._dragging and follow:
            self.x += (target_x - self.x) * self.lerp_factor
            self.y += (target_y - self.y) * self.lerp_factor

        # Clamp so camera never shows outside the world.
        # If the world is smaller than the viewport along an axis, the
        # clamp below is meaningless (half > world - half) - center the
        # camera on the world along that axis instead.
        half_w = (self.width / 2) / self.zoom
        half_h = (self.height / 2) / self.zoom
        self.x = world_w / 2 if world_w <= 2 * half_w else \
                 max(half_w, min(world_w - half_w, self.x))
        self.y = world_h / 2 if world_h <= 2 * half_h else \
                 max(half_h, min(world_h - half_h, self.y))

    def snap_to(self, x: float, y: float, world_w: float, world_h: float):
        """Instantly snap camera to a position (e.g., when pressing 'c')."""
        self.x = x
        self.y = y
        half_w = (self.width / 2) / self.zoom
        half_h = (self.height / 2) / self.zoom
        self.x = world_w / 2 if world_w <= 2 * half_w else \
                 max(half_w, min(world_w - half_w, self.x))
        self.y = world_h / 2 if world_h <= 2 * half_h else \
                 max(half_h, min(world_h - half_h, self.y))

    def world_to_screen(self, wx: float, wy: float) -> tuple[float, float]:
        sx = (wx - self.x) * self.zoom + self.width / 2
        sy = (self.y - wy) * self.zoom + self.height / 2   # north = up
        return sx, sy

    def screen_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        """Inverse of world_to_screen (needed by the obstacle palette to
        know where on the map the cursor is)."""
        wx = (sx - self.width / 2) / self.zoom + self.x
        wy = self.y - (sy - self.height / 2) / self.zoom   # north = up
        return wx, wy

    def zoom_in(self):
        self.zoom = min(self.zoom * config.ZOOM_STEP, config.MAX_ZOOM)

    def zoom_out(self):
        self.zoom = max(self.zoom / config.ZOOM_STEP, config.MIN_ZOOM)