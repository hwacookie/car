# Renderer
# Draws the road network, minimap, HUD and dashboard onto the Pygame surface.

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
        self._font_large: pygame.font.Font | None = None
        self._font_unit: pygame.font.Font | None = None
        try:
            pygame.font.init()
            self._font = pygame.font.SysFont("monospace", 14, bold=True)
            self._font_large = pygame.font.SysFont("monospace", 42, bold=True)
            self._font_unit = pygame.font.SysFont("monospace", 16)
        except Exception:
            self._font = None
            self._font_large = None
            self._font_unit = None

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
        """Draw the breadcrumb trail showing where the car has driven."""
        if not hasattr(car, 'trail') or len(car.trail) < 2:
            return
        
        # Draw small dots for each breadcrumb
        for wx, wy in car.trail:
            sx, sy = self.camera.world_to_screen(wx, wy)
            # Draw a small cyan dot
            pygame.draw.circle(surface, (0, 255, 255), (int(sx), int(sy)), 3)

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

        # Use available fonts or create simple ones
        try:
            font_large = self._font_large if self._font_large else pygame.font.SysFont("arial", 64, bold=True)
            font_unit = self._font_unit if self._font_unit else pygame.font.SysFont("arial", 24, bold=True)
            font_small = self._font if self._font else pygame.font.SysFont("arial", 14)
        except:
            # Ultimate fallback - just draw text directly without fancy fonts
            font_large = font_unit = font_small = None

        if font_large:
            # Mode indicator (top)
            driver_name = car.driver.get_name() if car.driver else "NONE"
            mode_color = (0, 200, 100) if driver_name == "RAILS" else (100, 150, 255)
            txt_mode = font_unit.render(driver_name, True, mode_color)
            surface.blit(txt_mode, (panel_x + 10, panel_y + 5))
            txt_hint = font_small.render("(TAB)", True, (100, 100, 100))
            surface.blit(txt_hint, (panel_x + 140, panel_y + 5))

            # Large speed number
            txt_speed = font_large.render(f"{kmh}", True, color)
            surface.blit(txt_speed, (panel_x + 15, panel_y + 25))

            # Unit - larger and more visible
            txt_unit_render = font_unit.render("km/h", True, (200, 200, 200))
            surface.blit(txt_unit_render, (panel_x + 15, panel_y + 85))
        else:
            # Simple number display without fonts
            # Just draw the speed as circles/bars (fallback)
            pass

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
        indicator_y = panel_y + 15
        # Brake
        if hasattr(car, '_braking') and car._braking:
            pygame.draw.circle(surface, (255, 0, 0), (panel_x + panel_w - 25, indicator_y), 5)
            if font_small:
                txt = font_small.render("B", True, (255, 255, 255))
                surface.blit(txt, (panel_x + panel_w - 29, indicator_y - 6))
        # Accel
        if hasattr(car, '_accelerating') and car._accelerating:
            pygame.draw.circle(surface, (0, 255, 0), (panel_x + panel_w - 50, indicator_y), 5)
            if font_small:
                txt = font_small.render("A", True, (255, 255, 255))
                surface.blit(txt, (panel_x + panel_w - 54, indicator_y - 6))
        # Blinker left
        if hasattr(car, 'blinker_left') and car.blinker_left:
            pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 75, indicator_y), 5)
            if font_small:
                txt = font_small.render("L", True, (255, 255, 255))
                surface.blit(txt, (panel_x + panel_w - 78, indicator_y - 6))
        # Blinker right
        if hasattr(car, 'blinker_right') and car.blinker_right:
            pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 100, indicator_y), 5)
            if font_small:
                txt = font_small.render("R", True, (255, 255, 255))
                surface.blit(txt, (panel_x + panel_w - 104, indicator_y - 6))

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

        if self._font is not None:
            speed_kmh = int(car.speed * 3.6)
            txt = self._font.render(f"{speed_kmh} km/h", True, (255, 255, 255))
            surface.blit(txt, (mm_x + 6, mm_y + mm_h - 22))