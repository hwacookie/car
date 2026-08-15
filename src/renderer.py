# Renderer
# Draws the road network, minimap, HUD and dashboard onto the Pygame surface.

from __future__ import annotations

import colorsys
import math
import pygame

from . import config
from .road_network import RoadNetwork
from .camera import Camera


def make_grass_background(w: int, h: int, tile_size: int = 32) -> pygame.Surface:
    """Pre-render a tiled grass texture for the screen background, modeled
    on the RQ 31 reference SVG: base #4c702e with sparse lighter grass
    strokes (#6f963f) and a couple of small lighter/darker dots. Built
    once and blitted as the full background each frame - static, since
    the pattern is uniform and doesn't need to follow the camera."""
    tile = pygame.Surface((tile_size, tile_size), pygame.SRCALPHA)
    tile.fill((76, 112, 46, 255))                        # #4c702e base
    stroke = (111, 150, 63, 140)                          # #6f963f @ ~55%
    for (x1, y1, x2, y2) in (
        (2, 8, 7, 5), (17, 5, 21, 11), (25, 25, 31, 21),
        (5, 27, 8, 20), (20, 18, 28, 20),
    ):
        pygame.draw.line(tile, stroke, (x1, y1), (x2, y2), 1)
    pygame.draw.circle(tile, (120, 158, 72, 153), (10, 17), 2)  # #789e48 @ 60%
    pygame.draw.circle(tile, (54, 86, 37, 153), (28, 8), 1)     # #365625 @ 60%

    bg = pygame.Surface((w, h))
    for ty in range(0, h, tile_size):
        for tx in range(0, w, tile_size):
            bg.blit(tile, (tx, ty))
    return bg


class Renderer:
    # Cache of PIL fonts by size (class-level, shared across instances)
    _pil_fonts: dict[int, object] = {}

    def __init__(self, network: RoadNetwork, camera: Camera):
        self.network = network
        self.camera = camera
        # HUD text is re-rendered via PIL only every Nth frame (~0.1s at
        # 60fps) - text rasterization is the most expensive part of the
        # HUD, and a 10 Hz speed readout is indistinguishable from 60 Hz.
        self._hud_frame = 0
        self._HUD_TEXT_INTERVAL = 6
        self._hud_texts: dict[str, pygame.Surface] = {}
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
        """Draw the road network as filled polygons with properly rounded
        bends and caps.

        Rather than hand-rolling trigonometry for each junction corner
        (which turned into an unreliable mess of "fillet" hacks), the
        roads are built with the standard, well-tested approach for this
        exact problem: treat each road as a stroked line and use
        Shapely's `buffer()` with round joins/caps. Contiguous same-width
        segments are merged (via `linemerge`) into one continuous line
        first, so a 90-degree bend becomes a single interior vertex of
        one LineString - buffering that naturally produces a smooth
        circular arc on the outer side and a clean single point on the
        inner side, exactly like a normal road/curb. This is computed
        once (cached) since the road network never changes at runtime.
        """
        w, h = surface.get_size()

        for color, polys in self.network.get_road_polygons_by_color():
            for ext, holes in polys:
                screen_pts = [self.camera.world_to_screen(x, y) for x, y in ext]
                xs = [p[0] for p in screen_pts]
                ys = [p[1] for p in screen_pts]
                if max(xs) < 0 or min(xs) > w or max(ys) < 0 or min(ys) > h:
                    continue
                pygame.draw.polygon(surface, color, [(int(x), int(y)) for x, y in screen_pts])
                # Punch out any holes (e.g. a roundabout's island) with
                # the background color - pygame can't fill a polygon
                # with a hole in one call, so paint the exterior first
                # and then re-paint each hole over it.
                for hole in holes:
                    hole_pts = [self.camera.world_to_screen(x, y) for x, y in hole]
                    pygame.draw.polygon(surface, config.BG_COLOR, [(int(x), int(y)) for x, y in hole_pts])

        self.draw_road_markings(surface)

    def draw_road_markings(self, surface: pygame.Surface):
        """Dashed white centerline down the middle of each road (the same
        merged, corner-rounded centerlines the paved-area polygons are
        buffered from - see RoadNetwork.get_centerlines() - so the dashes
        follow the actual curve through a bend instead of cutting
        straight across it), plus a single white dot at the middle of
        every real (3+-way) junction instead of trying to dash through
        the intersection itself."""
        w, h = surface.get_size()
        zoom = self.camera.zoom
        pppm = config.PIXELS_PER_METER
        # Plain road centerlines vs. the Autobahn Leitlinie (RQ 31:
        # 6 m dash / 12 m gap).
        c_dash_px, c_gap_px = 3.0 * pppm, 3.0 * pppm
        l_dash_px, l_gap_px = 6.0 * pppm, 12.0 * pppm

        # Fade the markings out as zoom decreases: a dash's on-screen
        # length (dash_px * zoom) below ~1 px is just flickering noise.
        # Fully opaque at >= 4 px, fully transparent at <= 1 px.
        def dash_alpha(dash_px: float) -> int:
            d = dash_px * zoom
            return int(255 * max(0.0, min(1.0, (d - 1.0) / 3.0)))

        a_c = dash_alpha(c_dash_px)
        lane_marks = self.network.get_lane_markings()
        a_l = dash_alpha(l_dash_px) if lane_marks else 0
        if a_c > 0 or a_l > 0:
            # Any alpha < 255 needs a per-pixel-alpha overlay; full
            # opacity can draw straight onto the screen surface.
            need_overlay = (0 < a_c < 255) or (0 < a_l < 255)
            target = surface if not need_overlay else pygame.Surface((w, h), pygame.SRCALPHA)
            if a_c > 0:
                for coords in self.network.get_centerlines():
                    self._draw_dashed_polyline(target, coords, c_dash_px, c_gap_px, w, h, a_c)
            if a_l > 0:
                # Multi-lane one-way carriageways, RQ 31: narrow solid
                # median-side edge, dashed Leitlinie, broad solid
                # Breitstrich at the stop lane, guardrails on medians.
                # (get_centerlines() skips oneway roads, so no double
                # draw.)
                for style, coords, width_m in lane_marks:
                    if style == "dashed":
                        self._draw_dashed_polyline(target, coords, l_dash_px, l_gap_px, w, h, a_l)
                    elif style == "solid":
                        self._draw_solid_polyline(target, coords, w, h, a_l, width_m)
                    else:  # guardrail
                        self._draw_solid_polyline(
                            target, coords, w, h, a_l, 0.15, (183, 189, 186))
            if target is not surface:
                surface.blit(target, (0, 0))

        dot_radius_px = 4
        for node_id, degree in self.network.node_degree.items():
            if degree < 3:
                continue
            node_xy = self.network.nodes.get(node_id)
            if node_xy is None:
                continue
            sx, sy = self.camera.world_to_screen(*node_xy)
            if sx < -dot_radius_px or sx > w + dot_radius_px or \
               sy < -dot_radius_px or sy > h + dot_radius_px:
                continue
            pygame.draw.circle(surface, (255, 255, 255), (int(sx), int(sy)), dot_radius_px)

    def _draw_dashed_polyline(self, surface, coords, dash_px, gap_px, w, h,
                              alpha=255):
        """Walk a polyline (world coords) at constant arc length, drawing
        alternating dash/gap segments - works for the rounded-corner
        centerlines (many short segments approximating an arc) just as
        well as a single long straight stretch. `alpha` fades the dashes
        out at low zoom (see draw_road_markings); when it is < 255 the
        surface must be SRCALPHA."""
        period = dash_px + gap_px
        if period <= 0:
            return
        distance_into_period = 0.0  # where in the dash/gap cycle we are

        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]
            seg_len = math.hypot(x2 - x1, y2 - y1)
            if seg_len < 1e-9:
                continue
            ux, uy = (x2 - x1) / seg_len, (y2 - y1) / seg_len

            traveled = 0.0
            while traveled < seg_len:
                phase = distance_into_period % period
                drawing = phase < dash_px
                # Distance remaining in the current dash/gap phase
                remaining_in_phase = (dash_px - phase) if drawing else (period - phase)
                step = min(remaining_in_phase, seg_len - traveled)

                if drawing:
                    ax, ay = x1 + ux * traveled, y1 + uy * traveled
                    bx, by = x1 + ux * (traveled + step), y1 + uy * (traveled + step)
                    sax, say = self.camera.world_to_screen(ax, ay)
                    sbx, sby = self.camera.world_to_screen(bx, by)
                    if not ((sax < 0 and sbx < 0) or (sax > w and sbx > w) or
                            (say < 0 and sby < 0) or (say > h and sby > h)):
                        # Real lane markings are ~0.15m wide, not 2m.
                        line_w = max(1, int(0.15 * config.PIXELS_PER_METER * self.camera.zoom))
                        pygame.draw.line(surface, (255, 255, 255, alpha), (sax, say), (sbx, sby), line_w)

                traveled += step
                distance_into_period += step

    def _draw_solid_polyline(self, surface, coords, w, h, alpha=255,
                             width_m: float = 0.15,
                             color: tuple = (251, 251, 245)):
        """Draw a solid line along a world-coordinate polyline - the
        RQ 31 edge lines / Breitstrich / median guardrails of multi-lane
        carriageways (see RoadNetwork.get_lane_markings). width_m >= 0.25
        (the Breitstrich) is drawn at twice the normal line thickness.
        When alpha < 255 the surface must be SRCALPHA."""
        cam = self.camera
        thin_w = max(1, int(0.15 * config.PIXELS_PER_METER * cam.zoom))
        line_w = max(2, 2 * thin_w) if width_m >= 0.25 else thin_w
        for i in range(len(coords) - 1):
            ax, ay = cam.world_to_screen(*coords[i])
            bx, by = cam.world_to_screen(*coords[i + 1])
            if not ((ax < 0 and bx < 0) or (ax > w and bx > w) or
                    (ay < 0 and by < 0) or (ay > h and by > h)):
                pygame.draw.line(surface, (*color, alpha), (ax, ay), (bx, by), line_w)

    # --- Breadcrumb Trail ---
    # Fixed rainbow gradient (oldest -> newest) applied to the most recent
    # N breadcrumb arrows, so the direction of travel *and* recency are
    # both visible at a glance. Older arrows fall back to plain white.
    # Generated once as a smooth 50-step violet -> red gradient (hue
    # sweeping from ~300 deg down to 0 deg in HSV space).
    _RECENT_RAINBOW = [
        tuple(int(c * 255) for c in colorsys.hsv_to_rgb(
            (300 - 300 * i / 49) / 360.0, 1.0, 1.0
        ))
        for i in range(50)
    ]  # 50 steps, index 0 = violet (oldest of the batch) -> index -1 = red (most recent)
    
    def draw_trail(self, surface: pygame.Surface, car):
        """Draw the breadcrumb trail: small chevron ("v") arrows showing
        the car's heading at each recorded point, instead of plain dots.
        
        The most recent N points are colored with a fixed rainbow
        sequence (oldest-of-the-recent-batch -> violet, most recent ->
        red); everything older is drawn plain white.
        """
        if not hasattr(car, 'trail') or len(car.trail) < 2:
            return
        
        zoom = self.camera.zoom
        pppm = config.PIXELS_PER_METER
        arrow_len = 6 * zoom    # length of each arrow leg (screen px)
        # Half-width matches the car's own half-width, so the trail
        # visually represents the car's actual footprint at each point.
        arrow_half_w = (config.CAR_WIDTH / 2) * pppm * zoom
        
        n = len(car.trail)
        n_recent = len(self._RECENT_RAINBOW)
        
        for i, point in enumerate(car.trail):
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
            
            # Index from the end: 0 = most recent (last appended)
            age = n - 1 - i
            if age < n_recent:
                color = self._RECENT_RAINBOW[n_recent - 1 - age]
            else:
                color = (255, 255, 255)
            
            pygame.draw.line(surface, color, (left_x, left_y), (tip_x, tip_y), 2)
            pygame.draw.line(surface, color, (right_x, right_y), (tip_x, tip_y), 2)

    # --- HUD / Dashboard ---
    def _hud_text(self, key: str, text: str, size: int, color: tuple[int, int, int]) -> pygame.Surface:
        """Cached HUD text: re-rendered via PIL only every Nth frame.
        The previous surface is blitted in the meantime, so between
        re-renders the (up to 0.1s) stale text is shown."""
        self._hud_frame += 1
        cached = self._hud_texts.get(key)
        if cached is None or self._hud_frame % self._HUD_TEXT_INTERVAL == 0:
            cached = self._text_surface(text, size, color)
            self._hud_texts[key] = cached
        return cached

    def draw_hud(self, surface: pygame.Surface, car):
        """Draw speedometer HUD in bottom-left corner of main window."""
        # Panel background
        panel_w, panel_h = 220, 130
        panel_x, panel_y = 15, surface.get_height() - panel_h - 15
        pygame.draw.rect(surface, (20, 20, 20, 200), (panel_x, panel_y, panel_w, panel_h))
        pygame.draw.rect(surface, (80, 80, 80), (panel_x, panel_y, panel_w, panel_h), 2)

        # Optional short label (e.g. "2/3" for the test map row/col
        # currently under test) - set remotely via POST /label, shown
        # top-right of the main window so it doesn't crowd the speedometer.
        label = getattr(self, 'hud_label', None)
        if label:
            label_surf = self._hud_text("label", str(label), 28, (255, 255, 0))
            lx = surface.get_width() - config.MINIMAP_MARGIN - label_surf.get_width() - 10
            ly = config.MINIMAP_MARGIN + config.MINIMAP_SIZE + 10
            pygame.draw.rect(surface, (20, 20, 20, 200),
                              (lx - 10, ly, label_surf.get_width() + 20, label_surf.get_height() + 12))
            pygame.draw.rect(surface, (80, 80, 80),
                              (lx - 10, ly, label_surf.get_width() + 20, label_surf.get_height() + 12), 2)
            surface.blit(label_surf, (lx, ly + 6))

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

        surface.blit(self._hud_text("mode", driver_name, 20, mode_color), (panel_x + 10, panel_y + 5))
        surface.blit(self._hud_text("tab", "(TAB)", 12, (100, 100, 100)), (panel_x + 120, panel_y + 8))

        # Speed number (large) + unit
        surface.blit(self._hud_text("speed", f"{kmh}", 54, color), (panel_x + 10, panel_y + 28))
        surface.blit(self._hud_text("unit", "km/h", 20, (200, 200, 200)), (panel_x + 10, panel_y + 100))

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
            surface.blit(self._hud_text("ind_b", "B", 12, (255, 255, 255)), (panel_x + panel_w - 29, indicator_y - 7))
        # Accel
        if getattr(car, '_accelerating', False):
            pygame.draw.circle(surface, (0, 255, 0), (panel_x + panel_w - 50, indicator_y), 5)
            surface.blit(self._hud_text("ind_a", "A", 12, (255, 255, 255)), (panel_x + panel_w - 54, indicator_y - 7))
        # Blinker left
        if blinker_left:
            pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 75, indicator_y), 5)
            surface.blit(self._hud_text("ind_l", "L", 12, (255, 255, 255)), (panel_x + panel_w - 78, indicator_y - 7))
        # Blinker right
        if blinker_right:
            pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 100, indicator_y), 5)
            surface.blit(self._hud_text("ind_r", "R", 12, (255, 255, 255)), (panel_x + panel_w - 104, indicator_y - 7))

    # --- Minimap ---
    def draw_minimap(self, surface: pygame.Surface, car):
        mm_w = config.MINIMAP_SIZE
        mm_h = config.MINIMAP_SIZE
        mm_x = surface.get_width() - mm_w - config.MINIMAP_MARGIN
        mm_y = config.MINIMAP_MARGIN

        mm_rect = pygame.Rect(mm_x, mm_y, mm_w, mm_h)
        pygame.draw.rect(surface, config.MINIMAP_BG, mm_rect)
        pygame.draw.rect(surface, config.MINIMAP_BORDER, mm_rect, 2)

        # Clip all map content to the minimap box - at low zoom the
        # yellow viewport rectangle alone can be larger than the box.
        prev_clip = surface.get_clip()
        surface.set_clip(mm_rect)

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

        surface.set_clip(prev_clip)

        speed_kmh = int(car.speed * 3.6)
        surface.blit(self._hud_text("mm_speed", f"{speed_kmh} km/h", 14, (255, 255, 255)), (mm_x + 6, mm_y + mm_h - 22))