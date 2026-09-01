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

    # --- Static road tiling ---
    # The road network never changes at runtime, so instead of transforming
    # and re-drawing every (often huge, merged-corridor) road polygon on
    # EVERY frame - which cost ~35 ms/frame on the Kleinmachnow OSM map -
    # roads + markings + junction dots are rendered once per 512x512 screen-
    # pixel tile at the current zoom and cached. Per-frame cost drops to a
    # handful of blits; new tiles pay a one-time render when first visited.
    TILE_PX = 512
    _MAX_TILES = 96

    def __init__(self, network: RoadNetwork, camera: Camera):
        self.network = network
        self.camera = camera
        # Test confirmation flags (set via POST /flags): a GREEN flag at
        # the scenario's start and a RED flag at its end, both drawn as
        # pennants to the right of the road so the player can see exactly
        # where the test begins and stops. [x_px, y_px, heading_deg].
        # The end flag arrives as (segment, progress) first and is resolved
        # to a position by the main loop once the route covers that segment
        # (flag_red_pending), so it is visible from the START of the test.
        self.flag_green: list | None = None
        self.flag_red: list | None = None
        self.flag_red_pending: tuple | None = None
        # True (default): the red flag is the car's NAVIGATION destination
        # (park at it). False: visual-only marker - running turn tests end
        # when the car PASSES the flag, no parking involved (docs/TESTING.md).
        self.flag_red_nav: bool = True
        from collections import OrderedDict
        self._tiles: "OrderedDict[tuple, pygame.Surface]" = OrderedDict()
        self._tile_polys: list | None = None
        # HUD text: re-rendered via PIL only when its content CHANGES,
        # rate-limited to one change per key every few frames (<=50 ms lag
        # at 60 fps - indistinguishable from live); unchanged strings are
        # never re-rendered at all. See _hud_text for why the old
        # per-CALL counter was wrong.
        self._frame = 0
        self._HUD_TEXT_MIN_INTERVAL = 3
        self._hud_texts: dict[str, tuple] = {}
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
        self._frame += 1
        self.draw_roads(surface)
        self.draw_paved_edge(surface)
        self.draw_test_flags(surface)
        # Trail is drawn AFTER the car sprite (see main.py) so buckets
        # are visible at the car's edges rather than hidden underneath.
        if car is not None:
            # Minimap + HUD describe the CAR (position, speed, segment);
            # with no car on the map there is nothing to show.
            self.draw_minimap(surface, car)
            self.draw_hud(surface, car)
        self.draw_zoom_indicator(surface)

    def draw_zoom_indicator(self, surface: pygame.Surface):
        """Top-left 'zoom N.Nx' readout - always visible (even before a
        car exists), so the current camera zoom is never a mystery. The
        viewport width in metres makes the scale concrete."""
        view_w_m = surface.get_width() / self.camera.zoom / config.PIXELS_PER_METER
        text = f"zoom {self.camera.zoom:.1f}x  ({view_w_m:.0f} m wide)"
        surf = self._hud_text("zoom", text, 20, (220, 220, 220))
        pygame.draw.rect(surface, (20, 20, 20, 180),
                         (8, 8, surf.get_width() + 14, surf.get_height() + 12))
        surface.blit(surf, (15, 13))

    def draw_test_flags(self, surface: pygame.Surface):
        """Draw the green start / red end test flags.

        Top-down pennants in WORLD space (they scale with camera zoom like
        everything else). Each is placed to the RIGHT of the road at the
        given position/heading: offset past the kerb along the right
        perpendicular of the heading, so it sits on the grass, never on
        the carriageway.
        """
        if self.flag_green:
            self._draw_flag(surface, self.flag_green,
                            (0, 200, 80), (0, 90, 40))
        if self.flag_red:
            self._draw_flag(surface, self.flag_red,
                            (230, 50, 50), (120, 20, 20))

    def _draw_flag(self, surface: pygame.Surface, pos: list,
                   fill: tuple, outline: tuple):
        x, y, hdg = pos[0], pos[1], pos[2]
        rad = math.radians(hdg)
        fwd = (math.sin(rad), math.cos(rad))      # direction of travel
        right = (math.cos(rad), -math.sin(rad))   # right-hand side
        PPPM = config.PIXELS_PER_METER
        off_m = 3.0                               # well past the kerb
        bx, by = x + right[0] * off_m * PPPM, y + right[1] * off_m * PPPM
        # Solid pennant: base across the travel direction, apex pointing
        # away from the road.
        w = 1.3 * PPPM      # half base (m -> px)
        h = 2.2 * PPPM      # length toward the grass
        p1 = (bx - fwd[0] * w, by - fwd[1] * w)
        p2 = (bx + fwd[0] * w, by + fwd[1] * w)
        ap = (bx + right[0] * h, by + right[1] * h)
        sx, sy = self.camera.world_to_screen(bx, by)
        s1 = self.camera.world_to_screen(*p1)
        s2 = self.camera.world_to_screen(*p2)
        sa = self.camera.world_to_screen(*ap)
        pygame.draw.polygon(surface, fill, [(sx, sy), s1, s2, sa])
        pygame.draw.polygon(surface, outline, [(sx, sy), s1, s2, sa], 2)
    def draw_paved_edge(self, surface: pygame.Surface):
        """The WHITE outline of the exact paved-area polygon that defines
        BOTH the rendered road surface and the off-road check
        (RoadNetwork.get_paved_polygon) — always drawn (user decision
        2026-08-31: no more G-key toggle), so the visible asphalt edge and
        the game's road-boundary perception can never silently diverge.
        Fixed 2 screen px wide, like all pygame lines. Rings are cached
        once (the network is static) and culled by bounding box per frame."""
        poly = self.network.get_paved_polygon()
        if getattr(self, "_paved_edge_cache", None) is None:
            import numpy as np
            polys = (poly.geoms if poly.geom_type == "MultiPolygon"
                     else [poly])
            rings = []
            for p in polys:
                for ring in (p.exterior, *p.interiors):
                    arr = np.asarray(ring.coords, dtype=float)
                    if arr.size == 0:
                        continue
                    rings.append((arr, (float(arr[:, 0].min()),
                                        float(arr[:, 0].max()),
                                        float(arr[:, 1].min()),
                                        float(arr[:, 1].max()))))
            self._paved_edge_cache = rings
        cam = self.camera
        hw = cam.width / (2.0 * cam.zoom) + 60.0 / cam.zoom
        hh = cam.height / (2.0 * cam.zoom) + 60.0 / cam.zoom
        wx0, wx1 = cam.x - hw, cam.x + hw
        wy0, wy1 = cam.y - hh, cam.y + hh
        for arr, (ax0, ax1, ay0, ay1) in self._paved_edge_cache:
            if ax1 < wx0 or ax0 > wx1 or ay1 < wy0 or ay0 > wy1:
                continue
            sx = (arr[:, 0] - cam.x) * cam.zoom + cam.width / 2.0
            sy = (cam.y - arr[:, 1]) * cam.zoom + cam.height / 2.0
            pygame.draw.lines(surface, (255, 255, 255), True,
                              [(float(a), float(b)) for a, b in zip(sx, sy)],
                              2)

    # --- Roads (tile-cached) ---
    def _tile_geom(self):
        """Road polygons with precomputed world-space bounding boxes and
        numpy point arrays.

        The polygons themselves come from the network's cached builder:
        each road is a stroked line buffered with Shapely (round
        joins/caps), contiguous same-width segments merged first, so a
        90-degree bend becomes one smooth arc instead of a fillet hack.
        Merged corridors span whole towns (the biggest on Kleinmachnow
        has ~52k points), so per-tile rendering clips them to the tile
        rect BEFORE handing them to pygame - see _sh_clip().
        """
        if self._tile_polys is None:
            import numpy as np
            polys = []
            # One uniform color for every road type (config.ROAD_COLOR) -
            # the per-color grouping is only used to iterate the polygons.
            for _color, groups in self.network.get_road_polygons_by_color():
                for ext, holes in groups:
                    xs = [p[0] for p in ext]
                    ys = [p[1] for p in ext]
                    arr = np.asarray(ext, dtype=np.float64)
                    hole_arrs = [np.asarray(h, dtype=np.float64) for h in holes]
                    polys.append((config.ROAD_COLOR, arr, hole_arrs,
                                  (min(xs), min(ys), max(xs), max(ys))))
            self._tile_polys = polys
        return self._tile_polys

    @staticmethod
    def _sh_clip(pts, nx, ny, c):
        """Vectorized Sutherland-Hodgman clip of a closed polygon (Nx2)
        against the half-plane n·p >= c. Returns an (Mx2) array of the
        kept/interpolated vertices IN BOUNDARY ORDER.

        Per edge S->E: inside/inside -> E; inside/outside -> X;
        outside/inside -> X AND E (the interior end vertex is easy to
        forget - dropping it cuts corners); outside/outside -> nothing.
        Order is preserved by stacking per-edge items [X, E] and
        flattening row-major (edge order).
        """
        import numpy as np
        if len(pts) == 0:
            return pts
        p = np.vstack([pts, pts[:1]])   # close the ring: edge n-1 wraps to 0
        d = p[:, 0] * nx + p[:, 1] * ny - c
        din, dout = d[:-1] >= 0, d[1:] >= 0
        E = p[1:]
        cross = din ^ dout
        X = np.full_like(E, np.nan)
        if cross.any():
            di, dj = d[:-1][cross], d[1:][cross]
            t = di / (di - dj)
            X[cross] = p[:-1][cross] + t[:, None] * (E[cross] - p[:-1][cross])
        items = np.stack([X, E], axis=1)          # (n-1, 2, 2)
        valid = np.stack([cross, dout], axis=1)   # X when crossing; E when inside
        return items[valid] if valid.any() else np.empty((0, 2))

    @classmethod
    def _clip_rect(cls, pts, x0, y0, x1, y1):
        """Clip a closed polygon (Nx2 local coords) to an axis-aligned rect."""
        import numpy as np
        if len(pts) == 0:
            return pts
        pts = cls._sh_clip(pts, 1.0, 0.0, x0)     # x >= x0
        if len(pts) == 0:
            return pts
        pts = cls._sh_clip(pts, -1.0, 0.0, -x1)   # x <= x1
        if len(pts) == 0:
            return pts
        pts = cls._sh_clip(pts, 0.0, 1.0, y0)     # y >= y0
        if len(pts) == 0:
            return pts
        return cls._sh_clip(pts, 0.0, -1.0, -y1)  # y <= y1

    def draw_roads(self, surface: pygame.Surface):
        """Blit the cached road tiles covering the viewport."""
        w, h = surface.get_size()
        cam = self.camera
        zoom = cam.zoom
        zkey = round(zoom, 4)
        tw_world = self.TILE_PX / zoom          # tile size in world px
        x0 = cam.x - w / 2.0 / zoom
        y0 = cam.y - h / 2.0 / zoom              # south (min world y)
        x1 = cam.x + w / 2.0 / zoom
        y1 = cam.y + h / 2.0 / zoom              # north
        tx0, ty0 = int(math.floor(x0 / tw_world)), int(math.floor(y0 / tw_world))
        tx1 = int(math.floor((x1 - 1e-9) / tw_world))
        ty1 = int(math.floor((y1 - 1e-9) / tw_world))

        # Collect the visible tiles; missing ones are rendered at most two
        # per frame (closest to the viewport centre first). Without the cap,
        # a zoom change re-renders all ~6 visible tiles in one frame - a
        # 100-150 ms hitch. With it, new tiles fill in over a few frames.
        needed = []
        for ty in range(ty0, ty1 + 1):
            for tx in range(tx0, tx1 + 1):
                key = (zkey, tx, ty)
                tile = self._tiles.get(key)
                if tile is None:
                    cx_ = (tx + 0.5) * tw_world - cam.x
                    cy_ = (ty + 0.5) * tw_world - cam.y
                    needed.append((cx_ * cx_ + cy_ * cy_, key, tx, ty))
                else:
                    self._tiles.move_to_end(key)
        needed.sort()
        for _dist, key, tx, ty in needed[:2]:
            tile = self._render_tile(zkey, tx, ty)
            self._tiles[key] = tile
            while len(self._tiles) > self._MAX_TILES:
                self._tiles.popitem(last=False)

        for ty in range(ty0, ty1 + 1):
            for tx in range(tx0, tx1 + 1):
                tile = self._tiles.get((zkey, tx, ty))
                if tile is None:
                    continue   # not rendered yet this frame - bg shows
                sx = (tx * tw_world - cam.x) * zoom + w / 2.0
                sy = (cam.y - (ty + 1) * tw_world) * zoom + h / 2.0
                surface.blit(tile, (int(round(sx)), int(round(sy))))

    def _render_tile(self, zoom: float, tx: int, ty: int) -> pygame.Surface:
        """Render one road tile: background + all intersecting road
        polygons + lane markings + junction dots, at the given zoom.
        Purely a function of (zoom, tile coords) - the network is static."""
        import numpy as np
        tile = pygame.Surface((self.TILE_PX, self.TILE_PX))
        tile.fill(config.BG_COLOR)
        tw_world = self.TILE_PX / zoom
        wx0 = tx * tw_world
        wy_north = (ty + 1) * tw_world           # world y of the tile's top edge

        def xform(wx: float, wy: float):
            return ((wx - wx0) * zoom, (wy_north - wy) * zoom)

        # Tile rect in local coords, with a small margin so polygon edges
        # touching the tile boundary don't leave 1px seams.
        m = 2.0
        for color, arr, hole_arrs, box in self._tile_geom():
            if (box[2] < wx0 or box[0] > wx0 + tw_world
                    or box[3] < wy_north - tw_world or box[1] > wy_north):
                continue
            # local coords: x grows right, y grows DOWN (tile top = the
            # tile's NORTH edge wy_north)
            pts = np.column_stack((
                (arr[:, 0] - wx0) * zoom,
                (wy_north - arr[:, 1]) * zoom))
            pts = self._clip_rect(pts, -m, -m, self.TILE_PX + m, self.TILE_PX + m)
            if len(pts) >= 3:
                pygame.draw.polygon(tile, color,
                                    [(int(x), int(y)) for x, y in pts])
            # Punch out any holes (e.g. a roundabout's island) with the
            # background color - pygame can't fill a polygon with a hole
            # in one call.
            for harr in hole_arrs:
                hpts = self._clip_rect(
                    np.column_stack(((harr[:, 0] - wx0) * zoom,
                                     (wy_north - harr[:, 1]) * zoom)),
                    -m, -m, self.TILE_PX + m, self.TILE_PX + m)
                if len(hpts) >= 3:
                    pygame.draw.polygon(tile, config.BG_COLOR,
                                        [(int(x), int(y)) for x, y in hpts])

        self.draw_road_markings(tile, xform=xform)

        # Section numbers: each segment's index at its 50% point, so the
        # road on screen can be matched against log/test segment IDs.
        # Baked into the tile cache - rendered once per (zoom, tile), not
        # every frame.
        for i, seg in enumerate(self.network.segments):
            lx, ly = xform((seg.x1 + seg.x2) / 2.0, (seg.y1 + seg.y2) / 2.0)
            if -24 < lx < self.TILE_PX + 24 and -24 < ly < self.TILE_PX + 24:
                txt = self._hud_text(f"segnum_{i}", str(i), 14, (175, 180, 185))
                tile.blit(txt, (int(round(lx - txt.get_width() / 2.0)),
                                int(round(ly)) - 16))
        return tile

    def draw_road_markings(self, surface: pygame.Surface, xform=None):
        """Dashed white centerline down the middle of each road (the same
        merged, corner-rounded centerlines the paved-area polygons are
        buffered from - see RoadNetwork.get_centerlines() - so the dashes
        follow the actual curve through a bend instead of cutting
        straight across it), plus a single white dot at the middle of
        every real (3+-way) junction instead of trying to dash through
        the intersection itself."""
        if xform is None:
            xform = self.camera.world_to_screen
        w, h = surface.get_size()
        zoom = self.camera.zoom
        pppm = config.PIXELS_PER_METER
        # Dash patterns from config (shared with the GET /map export so
        # external renderers draw the same pattern - RQ 31: fine dashes
        # for lane dividers, even finer for the parking-lane boundary).
        c_dash_px = config.CENTER_DASH_M * pppm
        c_gap_px = config.CENTER_GAP_M * pppm
        l_dash_px = config.LANE_DASH_M * pppm
        l_gap_px = config.LANE_GAP_M * pppm
        p_dash_px = config.PARK_DASH_M * pppm
        p_gap_px = config.PARK_GAP_M * pppm

        # Fade the markings out as zoom decreases: a dash's on-screen
        # length (dash_px * zoom) below ~1 px is just flickering noise.
        # Fully opaque at >= 4 px, fully transparent at <= 1 px.
        def dash_alpha(dash_px: float) -> int:
            d = dash_px * zoom
            return int(255 * max(0.0, min(1.0, (d - 1.0) / 3.0)))

        a_c = dash_alpha(c_dash_px)
        lane_marks = self.network.get_lane_markings()
        a_l = dash_alpha(l_dash_px) if lane_marks else 0
        a_pd = dash_alpha(p_dash_px) if any(s == "p_dash" for s, *_ in lane_marks) else 0
        # One-way direction arrows fade with the same zoom rule as the
        # other markings (their ~3 m length sets the on-screen size).
        a_a = dash_alpha(3.0 * pppm)
        # Painted P marks in parking lanes (~2 m tall letters).
        park_marks = self.network.get_parking_marks()
        a_p = dash_alpha(2.0 * pppm) if park_marks else 0
        if a_c > 0 or a_l > 0 or a_pd > 0 or a_a > 0 or a_p > 0:
            # Any alpha < 255 needs a per-pixel-alpha overlay; full
            # opacity can draw straight onto the screen surface.
            need_overlay = (0 < a_c < 255) or (0 < a_l < 255) \
                or (0 < a_pd < 255) or (0 < a_a < 255) or (0 < a_p < 255)
            target = surface if not need_overlay else pygame.Surface((w, h), pygame.SRCALPHA)
            if a_c > 0:
                # Width-filtered set (>= CENTERLINE_MIN_WIDTH_M) - the
                # LaneGuard uses the unfiltered get_centerlines().
                for coords in self.network.get_marking_centerlines():
                    self._draw_dashed_polyline(target, coords, c_dash_px, c_gap_px,
                                               w, h, a_c, xform)
            if a_l > 0:
                # Multi-lane one-way carriageways, RQ 31: narrow solid
                # median-side edge, dashed Leitlinie, broad solid
                # Breitstrich at the stop lane, guardrails on medians.
                # (get_centerlines() skips oneway roads, so no double
                # draw.)
                for style, coords, width_m in lane_marks:
                    if style == "dashed":
                        self._draw_dashed_polyline(target, coords, l_dash_px,
                                                   l_gap_px, w, h, a_l, xform)
                    elif style == "p_dash":
                        self._draw_dashed_polyline(target, coords, p_dash_px,
                                                   p_gap_px, w, h, a_pd, xform)
                    elif style == "solid":
                        self._draw_solid_polyline(target, coords, w, h, a_l,
                                                  width_m, xform=xform)
                    else:  # guardrail
                        self._draw_solid_polyline(
                            target, coords, w, h, a_l, 0.15, (183, 189, 186),
                            xform=xform)
            if a_a > 0:
                # Painted direction arrows on one-way roads: one per lane,
                # every 100 m, pointing along the legal direction.
                col = ((255, 255, 255, a_a) if need_overlay
                       else (255, 255, 255))
                for poly in self.network.get_oneway_arrows():
                    pygame.draw.polygon(target, col,
                                        [xform(*p) for p in poly])
            if a_p > 0:
                # Painted P marks in parking lanes (both kerbs on two-way
                # roads); they end >= PARK_LANE_END_GAP_M before junctions.
                col = ((255, 255, 255, a_p) if need_overlay
                       else (255, 255, 255))
                for poly in park_marks:
                    pygame.draw.polygon(target, col,
                                        [xform(*p) for p in poly])
            if target is not surface:
                surface.blit(target, (0, 0))

        # Junction dots: physical size (30 cm diameter) rendered in world
        # space, so they scale with zoom; 1 px floor keeps them visible
        # at low zoom.
        dot_radius_px = max(1, int(round(
            config.JUNCTION_DOT_RADIUS_M * config.PIXELS_PER_METER * zoom)))
        for node_id, degree in self.network.node_degree.items():
            if degree < 3:
                continue
            node_xy = self.network.nodes.get(node_id)
            if node_xy is None:
                continue
            sx, sy = xform(*node_xy)
            if sx < -dot_radius_px or sx > w + dot_radius_px or \
               sy < -dot_radius_px or sy > h + dot_radius_px:
                continue
            pygame.draw.circle(surface, (255, 255, 255), (int(sx), int(sy)), dot_radius_px)

    def _draw_dashed_polyline(self, surface, coords, dash_px, gap_px, w, h,
                              alpha=255, xform=None):
        """Walk a polyline (world coords) at constant arc length, drawing
        alternating dash/gap segments - works for the rounded-corner
        centerlines (many short segments approximating an arc) just as
        well as a single long straight stretch. `alpha` fades the dashes
        out at low zoom (see draw_road_markings); when it is < 255 the
        surface must be SRCALPHA. `xform` maps world px -> local px
        (camera transform per frame, tile-local transform for cached
        tiles)."""
        if xform is None:
            xform = self.camera.world_to_screen
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
            # Cull segments that lie entirely outside the target rect:
            # tiles are small while centerlines span whole towns. The dash
            # phase still advances so the pattern stays arc-length aligned.
            cx1, cy1 = xform(x1, y1)
            cx2, cy2 = xform(x2, y2)
            if ((cx1 < -8 and cx2 < -8) or (cx1 > w + 8 and cx2 > w + 8) or
                    (cy1 < -8 and cy2 < -8) or (cy1 > h + 8 and cy2 > h + 8)):
                distance_into_period = (distance_into_period + seg_len) % period
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
                    sax, say = xform(ax, ay)
                    sbx, sby = xform(bx, by)
                    if not ((sax < 0 and sbx < 0) or (sax > w and sbx > w) or
                            (say < 0 and sby < 0) or (say > h and sby > h)):
                        # Real lane markings are ~0.15m wide, not 2m.
                        line_w = max(1, int(0.15 * config.PIXELS_PER_METER * self.camera.zoom))
                        pygame.draw.line(surface, (255, 255, 255, alpha), (sax, say), (sbx, sby), line_w)

                traveled += step
                distance_into_period += step

    def _draw_solid_polyline(self, surface, coords, w, h, alpha=255,
                             width_m: float = 0.15,
                             color: tuple = (251, 251, 245), xform=None):
        """Draw a solid line along a world-coordinate polyline - the
        RQ 31 edge lines / Breitstrich / median guardrails of multi-lane
        carriageways (see RoadNetwork.get_lane_markings). width_m >= 0.25
        (the Breitstrich) is drawn at twice the normal line thickness.
        When alpha < 255 the surface must be SRCALPHA."""
        if xform is None:
            xform = self.camera.world_to_screen
        cam = self.camera
        thin_w = max(1, int(0.15 * config.PIXELS_PER_METER * cam.zoom))
        line_w = max(2, 2 * thin_w) if width_m >= 0.25 else thin_w
        for i in range(len(coords) - 1):
            ax, ay = xform(*coords[i])
            bx, by = xform(*coords[i + 1])
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
        """Cached HUD text.

        A CHANGED string is re-rendered at most once every few frames per
        key (<=50 ms display lag); an UNCHANGED string is never re-rendered.

        The previous implementation incremented a counter per CALL and
        gated on `counter % N == 0`. Since the number of calls per frame
        varies with which indicators are lit, that gate depended on the
        call count: when it was a multiple of N, any key whose phase did
        not land on 0 was NEVER re-rendered again - until some indicator
        toggled and changed the count. That is why the test-number label
        and one of the two speed readouts appeared to update 'at random
        times' mid-test instead of at test start.
        """
        entry = self._hud_texts.get(key)   # ((text,size,color), surface, frame)
        if entry is not None:
            content, surf, frame = entry
            if content == (text, size, color):
                return surf
            if self._frame - frame < self._HUD_TEXT_MIN_INTERVAL:
                return surf    # changed, but too soon - <=50 ms stale
        surf = self._text_surface(text, size, color)
        self._hud_texts[key] = ((text, size, color), surf, self._frame)
        return surf

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
        # Hazard lights (all four corners flash) - the car recognises it
        # cannot continue (or was told to via the REST API).
        if getattr(driver, 'hazard', False):
            import time as _time
            if (_time.time() % 0.5) <= 0.25:
                pygame.draw.circle(surface, (255, 180, 0), (panel_x + panel_w - 125, indicator_y), 5)
                surface.blit(self._hud_text("ind_h", "H", 12, (255, 255, 255)), (panel_x + panel_w - 128, indicator_y - 7))

        # Current segment ID (network index) - lets the player match what
        # they see on screen against test logs / expected end segments.
        surface.blit(self._hud_text("seg", f"seg {car.seg_idx}", 20,
                                    (160, 200, 255)),
                     (panel_x + panel_w - 95, panel_y + 100))

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
            color = config.ROAD_COLOR
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