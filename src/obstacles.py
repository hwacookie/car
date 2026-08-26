# Obstacles (docs/OBSTACLES.md, Part 1)
# Static parked-car obstacles: the shared placement logic used by BOTH the
# palette UI and the REST API (identical auto-alignment, identical off-road
# rejection), lane-direction auto-alignment from road geometry, the
# stop-on-contact collision response, and JSON save/load of obstacle layouts
# per map.

from __future__ import annotations

import json
import math
import os
import re
import threading
from dataclasses import dataclass, replace
from datetime import datetime, timezone

from . import config


# Palette colors for the parked car. The concrete RGB values are a rendering
# detail (docs/OBSTACLES.md): they must stay clearly distinguishable from
# the player's red #B41E1E and from each other.
OBSTACLE_COLORS: dict[str, tuple[int, int, int]] = {
    "blue":   (65, 105, 220),
    "yellow": (235, 195, 45),
    "white":  (238, 238, 238),
}

# Ghost tint for an invalid (off-road) drop target.
GHOST_INVALID_RGB = (230, 60, 60)


class PlacementError(ValueError):
    """An obstacle cannot be placed there / at all (off-road, bad type/color)."""


@dataclass(frozen=True)
class Obstacle:
    """A world object with position (x, y), heading, type and color.

    (x, y) is the CENTER of the footprint in world pixels; heading in
    degrees (0 = north), auto-aligned at placement from the road geometry.
    The id is stable for the life of the obstacle - required for removal
    via the REST API and unambiguous in saved layouts.
    """
    id: int
    x: float
    y: float
    heading: float
    type: str = "car"
    color: str = "blue"

    def to_dict(self) -> dict:
        return {"id": self.id, "type": self.type, "color": self.color,
                "x": self.x, "y": self.y, "heading": self.heading}

    @classmethod
    def from_dict(cls, d: dict) -> "Obstacle":
        return cls(id=int(d["id"]), x=float(d["x"]), y=float(d["y"]),
                   heading=float(d["heading"]),
                   type=str(d.get("type", "car")),
                   color=str(d.get("color", "blue")))


# --- Geometry helpers (world pixels, north-up frame) -----------------------

def box_corners(cx: float, cy: float, heading_deg: float,
                length_m: float, width_m: float) -> list[tuple[float, float]]:
    """Four corners of an oriented rectangle centered at (cx, cy)."""
    pppm = config.PIXELS_PER_METER
    h = math.radians(heading_deg)
    fx, fy = math.sin(h), math.cos(h)      # forward
    rx, ry = math.cos(h), -math.sin(h)     # right
    hl = length_m * pppm / 2.0
    hw = width_m * pppm / 2.0
    return [
        (cx + fx * hl + rx * hw, cy + fy * hl + ry * hw),   # front-right
        (cx + fx * hl - rx * hw, cy + fy * hl - ry * hw),   # front-left
        (cx - fx * hl - rx * hw, cy - fy * hl - ry * hw),   # rear-left
        (cx - fx * hl + rx * hw, cy - fy * hl + ry * hw),   # rear-right
    ]


def obstacle_footprint(ob: Obstacle) -> list[tuple[float, float]]:
    """The parked car's footprint: same size as the player car."""
    return box_corners(ob.x, ob.y, ob.heading,
                       config.CAR_LENGTH, config.CAR_WIDTH)


def player_body_corners(car) -> list[tuple[float, float]]:
    """The player car's body box - the same four-corner geometry the
    on-road check uses (centered on the body centre, not the rear axle)."""
    bx, by = car.body_center()
    return box_corners(bx, by, car.heading,
                       config.CAR_LENGTH, config.CAR_WIDTH)


def _project_onto(corners: list[tuple[float, float]], nx: float, ny: float):
    d = [c[0] * nx + c[1] * ny for c in corners]
    return min(d), max(d)


def boxes_intersect(a: list[tuple[float, float]],
                    b: list[tuple[float, float]]) -> bool:
    """SAT overlap test for two convex quads. Touching edges count as
    contact (separation must be strict), so a car resting flush against an
    obstacle still registers as in contact."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            nx, ny = -(y2 - y1), (x2 - x1)      # edge normal
            length = math.hypot(nx, ny)
            if length < 1e-12:
                continue
            nx, ny = nx / length, ny / length
            amin, amax = _project_onto(a, nx, ny)
            bmin, bmax = _project_onto(b, nx, ny)
            if amax < bmin or bmax < amin:
                return False                    # separating axis found
    return True


def point_in_box(px: float, py: float,
                 corners: list[tuple[float, float]]) -> bool:
    """Point-in-convex-quad test (cross products all on one side)."""
    n = len(corners)
    first = None
    for i in range(n):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % n]
        cr = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
        if abs(cr) < 1e-9:
            continue                            # on an edge: inside
        sign = cr > 0
        if first is None:
            first = sign
        elif sign != first:
            return False
    return True


# --- Auto-alignment ---------------------------------------------------------

def _nearest_segment(x: float, y: float, network):
    """(index, segment) of the road chord closest to world point (x, y),
    or (-1, None) if the network has no usable segments."""
    best_idx = -1
    best_dist = float("inf")
    for idx, seg in enumerate(network.segments):
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-9:
            continue
        t = ((x - seg.x1) * dx + (y - seg.y1) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        px = seg.x1 + t * dx - x
        py = seg.y1 + t * dy - y
        d = px * px + py * py
        if d < best_dist:
            best_dist = d
            best_idx = idx
    if best_idx < 0:
        return -1, None
    return best_idx, network.segments[best_idx]


def _lane_heading_chord(x: float, y: float, network) -> float:
    """Fallback alignment: the nearest segment's start->end direction,
    flipped for the oncoming half of a two-way road. Only used when no
    smoothed-centerline index is available (see lane_heading_at)."""
    idx, seg = _nearest_segment(x, y, network)
    if idx < 0:
        raise PlacementError("no road found at drop point")
    dx = seg.x2 - seg.x1
    dy = seg.y2 - seg.y1
    heading_fwd = math.degrees(math.atan2(dx, dy)) % 360.0
    if seg.oneway:
        return heading_fwd
    length_sq = dx * dx + dy * dy
    t = ((x - seg.x1) * dx + (y - seg.y1) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    px = x - (seg.x1 + t * dx)
    py = y - (seg.y1 + t * dy)
    h = math.radians(heading_fwd)
    side = px * math.cos(h) + py * (-math.sin(h))     # > 0: right half
    return heading_fwd if side >= 0 else (heading_fwd + 180.0) % 360.0


def _alignment_index(network):
    """Nearest-point index over the SMOOTHED centerlines (§10 geometry -
    the same splines and junction fillets that define the pavement),
    cached on the network. Returns (points, tangents) as float64 Nx2
    numpy arrays - one sample per ~0.5 m of road plus the (4x-subdivided)
    junction fillet arcs - or None when nothing could be built.

    The local direction of travel at any paved point is the tangent here,
    which is what makes parked cars follow curves and rounded corners
    instead of snapping to one of the straight segment chords."""
    cached = getattr(network, "_obstacle_alignment_cache", None)
    if cached is not None:
        return cached
    idx = None
    try:
        import numpy as np
        from .smooth_geometry import smoothed_network

        sm = smoothed_network(network)
        pts: list[tuple[float, float]] = []
        tans: list[tuple[float, float]] = []
        for line in sm.lines:
            res = line["resampled"]          # ~every 0.5 m of arc length
            n = len(res)
            if n < 2:
                continue
            curve = line["curve"]
            total = curve.total
            for i, (px, py) in enumerate(res):
                s = total * i / (n - 1)      # same s resample_curve used
                h = math.radians(curve.heading_at(s))
                pts.append((px, py))
                tans.append((math.sin(h), math.cos(h)))
        for fillet in sm.junction_fillets:
            arc = fillet["arc"]              # ordered along the turn
            if len(arc) < 3:
                continue
            sub: list[tuple[float, float]] = []
            for i in range(len(arc) - 1):
                x0, y0 = arc[i]
                x1, y1 = arc[i + 1]
                for k in range(4):
                    f = k / 4
                    sub.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
            sub.append(arc[-1])
            m = len(sub)
            for i in range(m):
                j0, j1 = max(0, i - 1), min(m - 1, i + 1)
                tx = sub[j1][0] - sub[j0][0]
                ty = sub[j1][1] - sub[j0][1]
                L = math.hypot(tx, ty)
                if L < 1e-9:
                    continue
                pts.append(sub[i])
                tans.append((tx / L, ty / L))
        if len(pts) >= 2:
            idx = (np.asarray(pts, dtype=np.float64),
                   np.asarray(tans, dtype=np.float64))
    except Exception:
        idx = None
    try:
        network._obstacle_alignment_cache = idx
    except Exception:
        pass
    return idx


def lane_heading_at(x: float, y: float, network) -> float:
    """Direction of travel (degrees) of the lane under world point (x, y).

    The local direction is the tangent of the smoothed centerline at the
    nearest paved point (§10 geometry), so on curves and rounded corners
    the parked car follows the street. On a two-way road the side of the
    centerline decides which way it faces: dropped in the right half it
    faces along the tangent, in the left/oncoming half against it - like a
    car stopped in traffic in that lane. On a one-way every lane flows the
    legal direction, so the tangent is oriented to match the nearest
    segment's start->end flow regardless of side.
    """
    idx_seg, seg = _nearest_segment(x, y, network)
    if idx_seg < 0:
        raise PlacementError("no road found at drop point")

    index = _alignment_index(network)
    if index is not None:
        pts, tans = index
        d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
        i = int(d2.argmin())
        qx, qy = float(pts[i][0]), float(pts[i][1])
        tx, ty = float(tans[i][0]), float(tans[i][1])
        if seg.oneway:
            fx = seg.x2 - seg.x1
            fy = seg.y2 - seg.y1
            L = math.hypot(fx, fy)
            sgn = 1.0
            if L > 1e-9 and tx * (fx / L) + ty * (fy / L) < 0:
                sgn = -1.0                     # face the legal flow
        else:
            rx, ry = ty, -tx                   # right of forward (tx, ty)
            side = (x - qx) * rx + (y - qy) * ry
            sgn = 1.0 if side >= 0 else -1.0   # > 0: right half
        h = math.degrees(math.atan2(sgn * tx, sgn * ty)) % 360.0
        return 0.0 if h >= 360.0 - 1e-9 else h

    return _lane_heading_chord(x, y, network)


# --- Manager ----------------------------------------------------------------

class ObstacleManager:
    """Owns the placed obstacles. Shared by the palette UI and the REST API
    so both paths go through the SAME placement logic (identical
    auto-alignment, identical off-road rejection). Thread-safe: the REST
    server thread mutates while the game loop reads every physics step."""

    def __init__(self, map_name: str, base_dir: str | None = None):
        self.map_name = map_name
        # Layouts live under <base>/obstacles/<map_name>/ (alongside the
        # existing data/osm_cache/). base_dir is overridable for tests.
        self.base_dir = base_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
        self.obstacles: list[Obstacle] = []
        self._next_id = 1
        self._lock = threading.Lock()

    # --- placement (identical for palette and REST) ---

    def align_heading(self, x: float, y: float, network) -> float:
        return lane_heading_at(x, y, network)

    @staticmethod
    def _validate_point(network, x: float, y: float):
        if not network.is_on_road(x, y):
            raise PlacementError(
                f"({x / config.PIXELS_PER_METER:.1f} m, "
                f"{y / config.PIXELS_PER_METER:.1f} m) is off the paved road area")

    def place(self, network, type: str, color: str,
              x: float, y: float) -> Obstacle:
        """Place a new obstacle; returns it (id + computed heading)."""
        if type != "car":
            raise PlacementError(
                f"unknown obstacle type '{type}' (Part 1 offers 'car' only)")
        if color not in OBSTACLE_COLORS:
            raise PlacementError(
                f"unknown color '{color}' "
                f"(available: {', '.join(sorted(OBSTACLE_COLORS))})")
        self._validate_point(network, x, y)
        heading = lane_heading_at(x, y, network)
        with self._lock:
            ob = Obstacle(id=self._next_id, x=float(x), y=float(y),
                          heading=heading, type=type, color=color)
            self._next_id += 1
            self.obstacles.append(ob)
        return ob

    def move(self, network, ob_id: int, x: float, y: float) -> Obstacle:
        """Re-place an existing obstacle (re-aligned at the new point)."""
        if not self.get(ob_id):
            raise KeyError(f"no obstacle with id {ob_id}")
        self._validate_point(network, x, y)
        heading = lane_heading_at(x, y, network)
        with self._lock:
            for i, o in enumerate(self.obstacles):
                if o.id == ob_id:
                    new_ob = replace(o, x=float(x), y=float(y), heading=heading)
                    self.obstacles[i] = new_ob
                    return new_ob
            raise KeyError(f"no obstacle with id {ob_id}")

    def remove(self, ob_id: int) -> bool:
        with self._lock:
            for i, o in enumerate(self.obstacles):
                if o.id == ob_id:
                    del self.obstacles[i]
                    return True
            return False

    def get(self, ob_id: int) -> Obstacle | None:
        with self._lock:
            for o in self.obstacles:
                if o.id == ob_id:
                    return o
            return None

    def snapshot(self) -> list[Obstacle]:
        """A consistent copy of the current set (thread-safe)."""
        with self._lock:
            return list(self.obstacles)

    def snapshot_dicts(self) -> list[dict]:
        with self._lock:
            return [o.to_dict() for o in self.obstacles]

    # --- Stop on contact (per-frame, all modes) ----------------------------

    def contact_with_car(self, car) -> Obstacle | None:
        """The first obstacle whose footprint touches the player's body box."""
        corners = player_body_corners(car)
        for ob in self.snapshot():
            if boxes_intersect(corners, obstacle_footprint(ob)):
                return ob
        return None

    def apply_contact_stop(self, car, dt: float,
                           pre_x: float, pre_y: float,
                           pre_heading: float | None = None) -> bool:
        """Stop-on-contact response (docs/OBSTACLES.md). Call after
        car.update() with the position AND heading from BEFORE that step.

        While the body box touches an obstacle footprint:
          1. the car brakes with full braking deceleration (A_BRAKE) until
             stopped - no instant velocity zeroing from speed;
          2. forward motion is clamped so the two boxes never interpenetrate
             - the car rests against the obstacle, like against a wall.
        Returns True while in contact (so the validator can treat the
        motion as externally constrained).
        """
        obs = self.snapshot()
        if not obs:
            return False

        pppm = config.PIXELS_PER_METER
        h = car.heading
        rad = math.radians(h)
        off_x = math.sin(rad) * config.REAR_AXLE_OFFSET_M * pppm
        off_y = math.cos(rad) * config.REAR_AXLE_OFFSET_M * pppm

        def body_at(x: float, y: float):
            return box_corners(x + off_x, y + off_y, h,
                               config.CAR_LENGTH, config.CAR_WIDTH)

        cur = body_at(car.x, car.y)
        if not any(boxes_intersect(cur, obstacle_footprint(o)) for o in obs):
            return False

        # 1) Brake to a stop - never an instant zeroing from speed. A tiny
        #    deadband zeroes the last floating-point crumbs (a residual of
        #    ~1e-15 m/s would otherwise persist forever against the
        #    sub-micron rest gap) so the car is truly at rest.
        if car.speed > 0.0:
            car.speed = max(0.0, car.speed - config.CAR_BRAKING * dt)
            if car.speed < 1e-6:
                car.speed = 0.0
        elif car.speed < 0.0:
            car.speed = min(0.0, car.speed + config.CAR_BRAKING * dt)
            if car.speed > -1e-6:
                car.speed = 0.0

        # 2) Clamp the motion of this step so the boxes never interpenetrate.
        pre_box = body_at(pre_x, pre_y)
        if not any(boxes_intersect(pre_box, obstacle_footprint(o)) for o in obs):
            # The pre-step position was clear: find the furthest point along
            # prev -> new that is still clear (the car rests against the
            # obstacle). A step (<= ~0.85 m at top speed) cannot skip over a
            # 4.4 m-wide obstacle, so overlap along the path is contiguous
            # and bisection converges to the contact point.
            lo, hi = 0.0, 1.0
            for _ in range(24):
                mid = (lo + hi) / 2.0
                mx = pre_x + (car.x - pre_x) * mid
                my = pre_y + (car.y - pre_y) * mid
                if any(boxes_intersect(body_at(mx, my), obstacle_footprint(o))
                       for o in obs):
                    hi = mid
                else:
                    lo = mid
            car.x = pre_x + (car.x - pre_x) * lo
            car.y = pre_y + (car.y - pre_y) * lo
        else:
            # The pre-step position already touches/overlaps. This is the
            # NORMAL resting state, not an error: after the bisection above
            # pins the car to the contact point, the pin is only clear by a
            # floating-point epsilon, and the next substep's heading
            # micro-adjustment (the driver always steers in tiny increments)
            # rotates the body box by ~REAR_AXLE_OFFSET*dh - enough to re-
            # register the pinned position as touching. Moving further into
            # the obstacle is impossible either way, so hold the pre-step
            # position while the braking above decays the speed: the car
            # rests against the obstacle like against a wall. (It also covers
            # an obstacle placed on top of the car - there the rollback is a
            # no-op and the car simply brakes in place.)
            car.x = pre_x
            car.y = pre_y
            # Holding position is not enough while the heading still moves:
            # the body box pivots around the axle offset, and its corners
            # would grind into the obstacle (interpenetration). If the held
            # position at the NEW heading intersects, hold the heading too -
            # the car is pinned. Steering that keeps the boxes clear is left
            # alone, so a driver can still steer back away from the contact.
            if pre_heading is not None and any(
                    boxes_intersect(body_at(pre_x, pre_y),
                                    obstacle_footprint(o)) for o in obs):
                car.heading = pre_heading
        return True

    # --- Save / load (obstacle layouts) ------------------------------------

    def layout_dir(self) -> str:
        return os.path.join(self.base_dir, "obstacles",
                            _sanitize_name(self.map_name))

    def list_layouts(self) -> list[str]:
        """Names of the saved layouts of THIS map (a Kleinmachnow layout is
        not loadable onto the synthetic basic map - the directory is per-map)."""
        d = self.layout_dir()
        if not os.path.isdir(d):
            return []
        return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json"))

    def save(self, name: str) -> str:
        """Store the current set under `name` (overwrites same-name)."""
        name = _sanitize_name(name)
        if not name:
            raise PlacementError("layout name must not be empty")
        d = self.layout_dir()
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{name}.json")
        with self._lock:
            obstacles = [o.to_dict() for o in self.obstacles]
        payload = {
            "map": self.map_name,
            "name": name,
            "saved_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "obstacles": obstacles,
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)
        return path

    def load(self, name: str, network) -> tuple[int, int]:
        """Replace the current obstacles with a saved layout. Each entry is
        validated against the paved polygon: an obstacle that no longer lies
        on the road (e.g. the map data changed) is skipped with a warning
        instead of being placed off-road. Returns (loaded, skipped)."""
        path = os.path.join(self.layout_dir(), f"{_sanitize_name(name)}.json")
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"no layout named '{name}' for map '{self.map_name}'")
        with open(path) as f:
            payload = json.load(f)
        if payload.get("map") != self.map_name:
            raise PlacementError(
                f"layout '{name}' belongs to map '{payload.get('map')}', "
                f"not '{self.map_name}'")
        new_obs: list[Obstacle] = []
        loaded = skipped = 0
        max_id = 0
        for entry in payload.get("obstacles", []):
            try:
                ob = Obstacle.from_dict(entry)
            except (KeyError, TypeError, ValueError):
                print(f"⚠️  Layout '{name}': skipping malformed entry {entry!r}")
                skipped += 1
                continue
            if ob.type != "car" or ob.color not in OBSTACLE_COLORS:
                print(f"⚠️  Layout '{name}': skipping unknown obstacle "
                      f"{ob.to_dict()}")
                skipped += 1
                continue
            if not network.is_on_road(ob.x, ob.y):
                print(f"⚠️  Layout '{name}': obstacle {ob.id} no longer lies on "
                      f"the paved road - skipping it")
                skipped += 1
                continue
            new_obs.append(ob)
            max_id = max(max_id, ob.id)
            loaded += 1
        with self._lock:
            self.obstacles = new_obs
            self._next_id = max_id + 1
        return loaded, skipped


def _sanitize_name(name: str) -> str:
    """Filesystem-safe layout/map name."""
    name = re.sub(r"[^A-Za-z0-9 _\-.]", "_", str(name)).strip()
    return name[:80]


# --- Recolored car sprites ---------------------------------------------------
# Obstacles are drawn like the player car: same sprite asset, recolored. The
# tint scales each pixel by its luminance relative to the player's red body
# (#d32f2f), so the body lands exactly on the target color while windows,
# wheels and lights keep their relative shading.

_SPRITE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "assets", "car_64x128.png")
_BASE_LUM = 0.299 * 0xD3 + 0.587 * 0x2F + 0.114 * 0x2F   # ≈ 96: the red body
_base_array = None
_sprite_cache: dict[tuple, object] = {}


def _get_base_array():
    global _base_array
    if _base_array is None:
        import numpy as np
        from PIL import Image
        with Image.open(_SPRITE_PATH) as im:
            _base_array = np.asarray(im.convert("RGBA"), dtype=np.float64)
    return _base_array


def tinted_car_sprite(rgb: tuple[int, int, int], alpha: int = 255):
    """The player's car sprite recolored to `rgb` (cached per color/alpha).
    `alpha < 255` pre-fades the whole sprite (the drag ghost is drawn at
    reduced opacity)."""
    import numpy as np
    import pygame
    key = (rgb, alpha)
    if key in _sprite_cache:
        return _sprite_cache[key]
    arr = _get_base_array().copy()
    lum = 0.299 * arr[..., 0] + 0.587 * arr[..., 1] + 0.114 * arr[..., 2]
    scale = np.where(arr[..., 3] > 0, lum / _BASE_LUM, 0.0)
    for c in range(3):
        arr[..., c] = np.clip(scale * rgb[c], 0, 255)
    if alpha < 255:
        arr[..., 3] = np.clip(arr[..., 3] * alpha / 255.0, 0, 255)
    h, w = arr.shape[:2]
    surf = pygame.image.fromstring(arr.astype(np.uint8).tobytes(), (w, h), "RGBA")
    _sprite_cache[key] = surf
    return surf
