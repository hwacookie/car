# Renderer
# Draws the road network, minimap, HUD and dashboard onto the Pygame surface.

from __future__ import annotations

import colorsys
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
        
        # Widened junction fillets: real intersections have a much wider
        # paved "corner-cutting" area than the roads' own width suggests
        # (curb radii flare the pavement out - confirmed via satellite
        # imagery, see docs/SPEC.md). A plain circle at the junction node
        # looks like an unnatural blob and can bulge into areas with no
        # actual road; instead, for each PAIR of adjacent roads (by angle
        # around the junction) we compute a proper tangent-arc fillet -
        # the same "circle tangent to two lines" geometry used for the
        # car's own turning arc, but with a fixed curb radius instead of a
        # speed-dependent one - and fill the resulting rounded wedge.
        # This naturally handles any junction degree (2-way corners,
        # 3-way, 4-way crossroads) and skips near-straight continuations
        # (no real corner to round off).
        for node_id, connected in self.network.node_connections.items():
            if len(connected) < 2:
                continue
            self._draw_junction_fillets(surface, node_id, connected, pppm, zoom, w, h)

    def _draw_junction_fillets(self, surface, node_id, connected, pppm, zoom, w, h):
        """Draw rounded curb-radius fillets in every 'corner' gap between
        adjacent roads meeting at a junction node."""
        node_xy = self.network.nodes.get(node_id)
        if node_xy is None:
            return
        node_x, node_y = node_xy
        
        # Spoke = direction AWAY from the junction along each connected
        # road, plus that road's own segment (for width/color).
        spokes = []
        for seg_idx in connected:
            seg = self.network.segments[seg_idx]
            if seg.start_node == node_id:
                away_dx, away_dy = seg.x2 - seg.x1, seg.y2 - seg.y1
            else:
                away_dx, away_dy = seg.x1 - seg.x2, seg.y1 - seg.y2
            length = math.hypot(away_dx, away_dy)
            if length < 1e-6:
                continue
            angle = math.atan2(away_dy, away_dx)
            spokes.append((angle, away_dx / length, away_dy / length, seg))
        
        if len(spokes) < 2:
            return
        spokes.sort(key=lambda s: s[0])
        
        MIN_GAP_DEG = 15.0   # skip near-parallel/duplicate spokes
        MAX_GAP_DEG = 155.0  # skip near-straight continuations (no real corner)
        
        n = len(spokes)
        for i in range(n):
            angle_a, ax, ay, seg_a = spokes[i]
            angle_b, bx, by, seg_b = spokes[(i + 1) % n]
            gap = (angle_b - angle_a) % (2 * math.pi)
            gap_deg = math.degrees(gap)
            if gap_deg < MIN_GAP_DEG or gap_deg > MAX_GAP_DEG:
                continue
            
            widest = seg_a if seg_a.width >= seg_b.width else seg_b
            radius_m = widest.width / 2 + config.JUNCTION_WIDENING_M
            
            half_gap = gap / 2
            tangent_dist_m = radius_m * math.tan(half_gap)
            center_dist_m = radius_m / math.cos(half_gap)
            
            bis_x, bis_y = ax + bx, ay + by
            bis_len = math.hypot(bis_x, bis_y)
            if bis_len < 1e-9:
                continue
            bis_x, bis_y = bis_x / bis_len, bis_y / bis_len
            
            pppm_full = pppm  # already includes PIXELS_PER_METER
            center_x = node_x + center_dist_m * pppm_full * bis_x
            center_y = node_y + center_dist_m * pppm_full * bis_y
            tangent1_x = node_x + tangent_dist_m * pppm_full * ax
            tangent1_y = node_y + tangent_dist_m * pppm_full * ay
            tangent2_x = node_x + tangent_dist_m * pppm_full * bx
            tangent2_y = node_y + tangent_dist_m * pppm_full * by
            
            start_angle = math.atan2(tangent1_y - center_y, tangent1_x - center_x)
            end_angle = math.atan2(tangent2_y - center_y, tangent2_x - center_x)
            sweep = (end_angle - start_angle) % (2 * math.pi)
            if sweep > math.pi:
                sweep -= 2 * math.pi
            
            # Build filled polygon: node -> tangent1 -> arc points -> tangent2 -> back to node
            radius_px = radius_m * pppm_full * zoom
            poly_world = [(node_x, node_y), (tangent1_x, tangent1_y)]
            num_arc_pts = 10
            for k in range(1, num_arc_pts + 1):
                a = start_angle + sweep * (k / num_arc_pts)
                ax_pt = center_x + radius_m * pppm_full * math.cos(a)
                ay_pt = center_y + radius_m * pppm_full * math.sin(a)
                poly_world.append((ax_pt, ay_pt))
            
            poly_screen = [self.camera.world_to_screen(px, py) for px, py in poly_world]
            
            # Cull if entirely off-screen
            xs = [p[0] for p in poly_screen]
            ys = [p[1] for p in poly_screen]
            if max(xs) < 0 or min(xs) > w or max(ys) < 0 or min(ys) > h:
                continue
            
            color = config.ROAD_TYPES.get(widest.highway, {}).get("color", (150, 150, 150))
            pygame.draw.polygon(surface, color, [(int(px), int(py)) for px, py in poly_screen])

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