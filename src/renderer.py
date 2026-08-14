# Renderer
# Draws the road network, minimap, HUD and dashboard onto the Pygame surface.

from __future__ import annotations

import math
import pygame

from . import config
from .road_network import RoadNetwork
from .camera import Camera


class Renderer:
    # Cache of PIL fonts by size (class-level, shared across instances)
    _pil_fonts: dict[int, object] = {}

    def __init__(self, network: RoadNetwork, camera: Camera):
        self.network = network
        self.camera = camera
        # Note: pygame.font is broken on some platforms (SDL_ttf import
        # issue). All text rendering goes through PIL instead — see
        # _text_surface() below.

    @classmethod
    def _get_pil_font(cls, size: int):
        """Load (and cache) a PIL font at the given point size."""
        from PIL import ImageFont
        if size not in cls._pil_fonts:
            try:
                cls._pil_fonts[size] = ImageFont.truetype(
                    "/System/Library/Fonts/Helvetica.ttc", size
                )
            except Exception:
                cls._pil_fonts[size] = ImageFont.load_default()
        return cls._pil_fonts[size]

    def _text_surface(self, text: str, size: int, color: tuple[int, int, int]) -> pygame.Surface:
        """Render text to a Pygame surface using PIL (pygame.font is unreliable)."""
        from PIL import Image, ImageDraw
        font = self._get_pil_font(size)
        # Measure text bounding box
        tmp = Image.new("RGBA", (1, 1), (0, 0, 0, 0))
        bbox = ImageDraw.Draw(tmp).textbbox((0, 0), text, font=font)
        w = max(1, bbox[2] - bbox[0] + 2)
        h = max(1, bbox[3] - bbox[1] + 2)
        img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.text((-bbox[0], -bbox[1]), text, fill=(*color, 255), font=font)
        return pygame.image.fromstring(img.tobytes(), img.size, img.mode)

    def draw(self, surface: pygame.Surface, car):
        self.draw_roads(surface)
        self.draw_trail(surface, car)  # Draw breadcrumb trail
        self.draw_minimap(surface, car)
        self.draw_hud(surface, car)

    # --- Roads ---
    def draw_roads(self, surface: pygame.Surface):
        """Draw every road segment as a filled polygon with rounded caps.
        Camera transform applied per-vertex — no texture scaling, no pixelation."""
        w, h = surface.get_size()
        zoom = self.camera.zoom
        pppm = config.PIXELS_PER_METER

        for seg in self.network.segments:
            color = config.ROAD_TYPES.get(seg.highway, {}).get("color", (150, 150, 150))
            half_w = (seg.width / 2) * pppm * zoom
            r = max(1, int(half_w))

            # Transform to screen coordinates
            sx1, sy1 = self.camera.world_to_screen(seg.x1, seg.y1)
            sx2, sy2 = self.camera.world_to_screen(seg.x2, seg.y2)

            # Cull if completely off-screen
            if (sx1 < -r and sx2 < -r) or (sx1 > w + r and sx2 > w + r) or \
               (sy1 < -r and sy2 < -r) or (sy1 > h + r and sy2 > h + r):
                continue

            # Compute perpendicular offset
            dx = sx2 - sx1
            dy = sy2 - sy1
            length = math.hypot(dx, dy)
            if length < 0.5:
                continue

            nx = -dy / length
            ny = dx / length

            # Build rectangle polygon
            pts = [
                (sx1 + nx * half_w, sy1 + ny * half_w),
                (sx1 - nx * half_w, sy1 - ny * half_w),
                (sx2 - nx * half_w, sy2 - ny * half_w),
                (sx2 + nx * half_w, sy2 + ny * half_w),
            ]
            pygame.draw.polygon(surface, color, [(int(p[0]), int(p[1])) for p in pts])

            # Rounded caps at both ends
            pygame.draw.circle(surface, color, (int(sx1), int(sy1)), r)
            pygame.draw.circle(surface, color, (int(sx2), int(sy2)), r)

    # --- Breadcrumb Trail ---
    def draw_trail(self, surface: pygame.Surface, car):
        """Draw the breadcrumb trail: small chevron ("v") arrows showing
        the car's heading at each recorded point, instead of plain dots.
        """
        if not hasattr(car, 'trail') or len(car.trail) < 2:
            return
        
        zoom = self.camera.zoom
        arrow_len = 6 * zoom    # length of each arrow leg (screen px)
        arrow_half_w = 4 * zoom  # half-width of the chevron opening
        
        for point in car.trail:
            # Trail points are (x, y, heading); tolerate old (x, y) tuples too
            if len(point) == 3:
                wx, wy, heading = point
            else:
                wx, wy = point
                heading = 0.0
            
            sx, sy = self.camera.world_to_screen(wx, wy)
            rad = math.radians(heading)
            
            # Forward direction (matches car movement convention: heading
            # 0 = screen "up"/away from viewer along +y world axis)
            fx, fy = math.sin(rad), -math.cos(rad)
            # Perpendicular (for the two chevron legs)
            px, py = -fy, fx
            
            tip_x, tip_y = sx + fx * arrow_len, sy + fy * arrow_len
            back_x, back_y = sx - fx * arrow_len, sy - fy * arrow_len
            left_x, left_y = back_x + px * arrow_half_w, back_y + py * arrow_half_w
            right_x, right_y = back_x - px * arrow_half_w, back_y - py * arrow_half_w
            
            pygame.draw.line(surface, (0, 255, 255), (left_x, left_y), (tip_x, tip_y), 2)
            pygame.draw.line(surface, (0, 255, 255), (right_x, right_y), (tip_x, tip_y), 2)

    # --- HUD / Dashboard ---
    def draw_hud(self, surface: pygame.Surface, car):
        """Draw speedometer HUD in bottom-left corner of main window."""
        # Panel background
        panel_w, panel_h = 220, 130
        panel_x, panel_y = 15, surface.get_height() - panel_h - 15
        pygame.draw.rect(surface, (20, 20, 20, 200), (panel_x, panel_y, panel_w, panel_h))
        pygame.draw.rect(surface, (80, 80, 80), (panel_x, panel_y, panel_w, panel_h), 2)

        # Speed number
        kmh = int(car.speed * 3.6)
        color = (255, 255, 255)
        if kmh > 50:
            color = (255, 200, 0)
        if kmh > 100:
            color = (255, 80, 80)

        # Mode indicator (top)
        driver_name = car.driver.get_name() if car.driver else "NONE"
        mode_color = (0, 200, 100) if driver_name == "RAILS" else (100, 150, 255)

        surface.blit(self._text_surface(driver_name, 20, mode_color), (panel_x + 10, panel_y + 5))
        surface.blit(self._text_surface("(TAB)", 12, (100, 100, 100)), (panel_x + 120, panel_y + 8))

        # Speed number (large) + unit
        surface.blit(self._text_surface(f"{kmh}", 54, color), (panel_x + 10, panel_y + 28))
        surface.blit(self._text_surface("km/h", 20, (200, 200, 200)), (panel_x + 10, panel_y + 100))

        # Speedometer arc (right side of panel)
        cx = panel_x + 170
        cy = panel_y + 70
        radius = 38
        max_kmh = 180
        # Background arc (grey ticks)
        for angle in range(0, 241, 6):
            rad = math.radians(angle + 150)
            x = cx + math.cos(rad) * radius
            y = cy + math.sin(rad) * radius
            pygame.draw.circle(surface, (60, 60, 60), (int(x), int(y)), 2)
        # Active arc (colored by speed)
        needle_angle = min(240, (kmh / max_kmh) * 240)
        for angle in range(0, int(needle_angle) + 1, 3):
            rad = math.radians(angle + 150)
            x = cx + math.cos(rad) * radius
            y = cy + math.sin(rad) * radius
            if angle < 120:
                c = (0, 200, 0)
            elif angle < 200:
                c = (255, 200, 0)
            else:
                c = (255, 60, 60)
            pygame.draw.circle(surface, c, (int(x), int(y)), 3)

        # Indicators: Brake (red), Accel (green), Blinkers (orange)
        # Driver may expose braking/accelerating flags directly (Car) or
        # blinker state indirectly (via car.driver for AIDriver).
        indicator_y = panel_y + 15
        driver = getattr(car, 'driver', None)
        blinker_left = getattr(driver, 'blinker_left', False)
        blinker_right = getattr(driver, 'blinker_right', False)

        # Brake
        if getattr(car, '_braking', False):
            pygame.draw.circle(surface, (255, 0, 0), (panel_x + panel_w - 25, indicator_y), 5)
            surface.blit(self._text_surface("B", 12, (255, 255, 255)), (panel_x + panel_w - 29, indicator_y - 7))
        # Accel
        if getattr(car, '_accelerating', False):
            pygame.draw.circle(surface, (0, 255, 0), (panel_x + panel_w - 50, indicator_y), 5)
            surface.blit(self._text_surface("A", 12, (255, 255, 255)), (panel_x + panel_w - 54, indicator_y - 7))
        # Blinker left
        if blinker_left:
            pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 75, indicator_y), 5)
            surface.blit(self._text_surface("L", 12, (255, 255, 255)), (panel_x + panel_w - 78, indicator_y - 7))
        # Blinker right
        if blinker_right:
            pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 100, indicator_y), 5)
            surface.blit(self._text_surface("R", 12, (255, 255, 255)), (panel_x + panel_w - 104, indicator_y - 7))

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
            y1 = mm_y + mm_h - seg.y1 * sy
            x2 = mm_x + seg.x2 * sx
            y2 = mm_y + mm_h - seg.y2 * sy
            pygame.draw.line(surface, color, (x1, y1), (x2, y2), 1)

        cx = mm_x + car.x * sx
        cy = mm_y + mm_h - car.y * sy
        pygame.draw.circle(surface, config.MINIMAP_CAR_COLOR, (int(cx), int(cy)), 3)

        # Viewport rectangle on minimap (yellow outline)
        vp_w = (surface.get_width() / self.camera.zoom) * sx
        vp_h = (surface.get_height() / self.camera.zoom) * sy
        vp_x = mm_x + (self.camera.x - (surface.get_width() / 2) / self.camera.zoom) * sx
        vp_y = mm_y + mm_h - (self.camera.y + (surface.get_height() / 2) / self.camera.zoom) * sy
        pygame.draw.rect(surface, (255, 255, 0), (int(vp_x), int(vp_y), int(vp_w), int(vp_h)), 2)

        speed_kmh = int(car.speed * 3.6)
        surface.blit(self._text_surface(f"{speed_kmh} km/h", 14, (255, 255, 255)), (mm_x + 6, mm_y + mm_h - 22))