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
        # Trail is drawn AFTER the car sprite (see main.py) so buckets
        # are visible at the car's edges rather than hidden underneath.
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
    # Paint bucket colors: small filled markers placed at each of the
    # car's four corners for every recorded trail point. Front buckets
    # are yellow, rear buckets are blue, so the swept footprint (and its
    # width) is visible at a glance. The bucket diameter equals the tire
    # width (~0.2 m), so each bucket is exactly the contact patch of one
    # tire.
    _BUCKET_FL = (255, 80, 80)       # red     — front left
    _BUCKET_FR = (80, 255, 80)       # green   — front right
    _BUCKET_RL = (80, 140, 255)      # blue    — rear left
    _BUCKET_RR = (255, 220, 0)       # yellow  — rear right
    # Buckets sit on the car's side edges (not beyond the body) and are
    # this far in from the front / rear edges.
    _BUCKET_INSET_M = 0.5            # m from front/rear edge

    def draw_trail(self, surface: pygame.Surface, car):
        """Draw the breadcrumb trail: one continuous polyline through all
        recorded positions (no gaps, no discrete arrow shapes), plus a
        small filled "paint bucket" at each of the car's four corners
        for every recorded point. The buckets are placed on the car's
        side edges (_BUCKET_INSET_M in from the front and rear edges),
        transformed with the recorded center and heading: each tire has
        its own color (FL=red, FR=green, RL=blue, RR=yellow).
        """
        trail = getattr(car, 'trail', None)
        if not trail or len(trail) < 2:
            return

        pppm = config.PIXELS_PER_METER
        # Trail points record the REAR AXLE (Car.x/y = the bicycle model's
        # pivot), so the rear tyres sit ON the recorded point and the front
        # tyres one wheelbase ahead of it. Offsetting the rear tyres
        # backwards from it (as this used to do) draws them behind the real
        # pivot, where they visibly swing OUT of every turn.
        front_px = config.SPRITE_WHEELBASE_M * pppm
        out_px = config.TIRE_OUTBOARD_M * pppm

        # Screen position + heading for every recorded point. Trail
        # points are (x, y, heading); tolerate old (x, y) tuples too.
        points = []
        for point in trail:
            wx, wy = point[0], point[1]
            heading = point[2] if len(point) >= 3 else 0.0
            points.append((wx, wy, heading))

        # Collect screen-space tire tracks for each of the four wheels
        fl_track = []  # front-left (red)
        fr_track = []  # front-right (green)
        rl_track = []  # rear-left (blue)
        rr_track = []  # rear-right (yellow)
        for wx, wy, heading in points:
            rad = math.radians(heading)
            fx, fy = math.sin(rad), math.cos(rad)    # forward
            rx, ry = math.cos(rad), -math.sin(rad)   # right
            fl_track.append(self.camera.world_to_screen(
                wx + front_px * fx - out_px * rx,
                wy + front_px * fy - out_px * ry))
            fr_track.append(self.camera.world_to_screen(
                wx + front_px * fx + out_px * rx,
                wy + front_px * fy + out_px * ry))
            rl_track.append(self.camera.world_to_screen(
                wx - out_px * rx, wy - out_px * ry))
            rr_track.append(self.camera.world_to_screen(
                wx + out_px * rx, wy + out_px * ry))

        # Draw one continuous line per tire track
        for track, color in ((fl_track, self._BUCKET_FL), (fr_track, self._BUCKET_FR),
                             (rl_track, self._BUCKET_RL), (rr_track, self._BUCKET_RR)):
            pygame.draw.lines(surface, color, False, [tuple(int(c) for c in p) for p in track], 2)

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
        mode_color = (0, 200, 100) if driver_name == "BICYCLE" else (100, 150, 255)

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
        # blinker state indirectly (via car.driver for BicycleDriver).
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