#!/usr/bin/env python3
"""Bicycle-model car prototype (see docs/TURN_REWORK_PLAN.md, section 6).

A ONE-CAR, kinematic bicycle model that replaces the rail model. The car
is a free particle on the road SURFACE (not a train on the OSM graph):

    state = (x, y, heading, v, steering_delta)
    heading 0 = north (+y), forward = (sin h, cos h), positive steer = right

Physics:
    v          += (throttle - brake) * dt          (longitudinal)
    d(heading) = (v / L) * tan(delta) * dt         (bicycle kinematics)
    x         += v * sin(heading) * dt
    y         += v * cos(heading) * dt

Understeer is EMERGENT: the heading rate is capped by a lateral-accel
limit (a_lat = v * |d(heading)/dt| <= A_LAT_MAX), so at speed the car
cannot turn as sharply and swings wide instead of teleporting.

Driver (two-tier, per the plan):
    intent  - a precomputed speed profile (v_max at each arc-length point,
              from the reference line's curvature and a look-back braking
              constraint) + a pure-pursuit steering target.
    control - per frame: throttle/brake to the profile, steer to the
              pursuit target (lookahead proportional to speed).

Reference line: the route's centerline, arc-length parameterized. The car
must STAY ON THE ROAD (network.is_on_road, the paved polygon). Success =
on-road through several junctions incl. tight corners, with no rail logic.

Standalone: reuses src.road_network geometry only, does NOT touch src/.

Run:
    .venv/bin/python scripts/proto_bicycle.py                 # OSM map
    .venv/bin/python scripts/proto_bicycle.py --map basic     # synthetic map
    .venv/bin/python scripts/proto_bicycle.py --map basic --start sliver_approach
    .venv/bin/python scripts/proto_bicycle.py --render out.png
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field

# --- Make `src` importable when run as a script from the repo root ---
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src import config
from src.road_network import RoadNetwork, _round_polyline_corners
from src.test_maps import build_test_map

PPPM = config.PIXELS_PER_METER
DT = 1.0 / 60.0


# ======================================================================
# Reference line (route centerline, arc-length parameterized)
# ======================================================================

@dataclass
class RefLine:
    """An arc-length-parameterized polyline (the route's centerline)."""
    pts: list[tuple[float, float]]          # world px
    seglen: list[float]                     # per-edge length (m)
    cum: list[float]                        # cumulative arc length (m)
    total: float                            # total length (m)

    @classmethod
    def from_points(cls, pts: list[tuple[float, float]]) -> "RefLine":
        seglen = [math.hypot(pts[i + 1][0] - pts[i][0],
                             pts[i + 1][1] - pts[i][1]) / PPPM
                  for i in range(len(pts) - 1)]
        cum = [0.0]
        for L in seglen:
            cum.append(cum[-1] + L)
        return cls(pts=pts, seglen=seglen, cum=cum, total=cum[-1])

    def point_at(self, s: float) -> tuple[float, float]:
        """World-px position at arc length s (clamped to [0, total])."""
        s = max(0.0, min(self.total, s))
        # binary search for the segment
        lo, hi = 0, len(self.cum) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(0, lo - 1)
        L = self.seglen[i]
        t = 0.0 if L < 1e-9 else (s - self.cum[i]) / L
        x = self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0])
        y = self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1])
        return x, y

    def heading_at(self, s: float) -> float:
        """Heading (deg, 0=north) of the reference line at arc length s."""
        s = max(0.0, min(self.total - 1e-6, s))
        lo, hi = 0, len(self.cum) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(0, lo - 1)
        dx = self.pts[i + 1][0] - self.pts[i][0]
        dy = self.pts[i + 1][1] - self.pts[i][1]
        return math.degrees(math.atan2(dx, dy))

    def curvature_at(self, s: float) -> float:
        """Signed curvature (1/m) at arc length s: + = right turn, - = left.

        A finite-difference over a small window so it is stable on the
        (piecewise-linear) reference line.
        """
        h = max(1.0, self.total * 0.01)
        h0 = math.radians(self.heading_at(s - h))
        h1 = math.radians(self.heading_at(s + h))
        dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
        return dh / (2 * h)


# ======================================================================
# Route (a sequence of directed segments through the road graph)
# ======================================================================

def _directed_edges(network: RoadNetwork) -> dict[tuple[str, str], int]:
    """Map (from_node, to_node) -> segment index for legal travel directions."""
    edges = {}
    for idx, seg in enumerate(network.segments):
        edges[(seg.start_node, seg.end_node)] = idx
        if not seg.oneway:
            edges[(seg.end_node, seg.start_node)] = idx
    return edges


def _exit_angle_deg(network: RoadNetwork, from_node: str, to_node: str,
                    approach_dir: tuple[float, float]) -> float:
    """Turning angle (deg, + = right) when arriving at `from_node` heading
    along `approach_dir` (unit vector) and leaving toward `to_node`."""
    fx, fy = network.nodes[from_node]
    tx, ty = network.nodes[to_node]
    dx, dy = tx - fx, ty - fy
    L = math.hypot(dx, dy) or 1.0
    dx, dy = dx / L, dy / L
    # angle between approach_dir and exit_dir, signed (+ = right/clockwise)
    cross = approach_dir[0] * dy - approach_dir[1] * dx   # >0 => exit is to the LEFT
    dot = approach_dir[0] * dx + approach_dir[1] * dy
    return -math.degrees(math.atan2(cross, dot))


def _approach_dir(network: RoadNetwork, node: str, toward_node: str) -> tuple[float, float]:
    """Unit vector of travel direction AS WE ARRIVE at `node` from
    `toward_node` (i.e. pointing from toward_node to node)."""
    ax, ay = network.nodes[toward_node]
    bx, by = network.nodes[node]
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy) or 1.0
    return dx / L, dy / L


def build_route(network: RoadNetwork, start_node: str,
                strategy: str = "straight", max_nodes: int = 400) -> list[str]:
    """Build a route (list of node ids) starting at `start_node`.

    strategy:
        "straight" - at each junction take the smallest-|angle| exit
                     (the straight continuation).
        "left" / "right" - prefer the sharpest left/right exit, else
                     fall back to straight.
    Respects one-way direction (only enters a oneway segment at its
    start_node). Avoids immediately reversing onto the segment we came
    from.
    """
    edges = _directed_edges(network)
    route: list[str] = [start_node]
    prev: str | None = None

    for _ in range(max_nodes):
        node = route[-1]
        conns = network.node_connections.get(node, [])
        # candidate outgoing neighbors (legal direction)
        cands = []
        for idx in conns:
            seg = network.segments[idx]
            other = seg.end_node if seg.start_node == node else seg.start_node
            if other == prev:
                continue  # don't reverse
            if (node, other) not in edges:
                continue  # illegal (oneway the wrong way)
            cands.append(other)
        if not cands:
            break  # dead end

        if len(cands) == 1:
            nxt = cands[0]
        else:
            appr = _approach_dir(network, node, prev) if prev else (0.0, 1.0)
            scored = [(_exit_angle_deg(network, node, c, appr), c) for c in cands]
            if strategy == "left":
                scored.sort(key=lambda x: x[0])          # most negative first
                best = scored[0][1]
                if scored[0][0] > -10:                    # no real left
                    best = min(scored, key=lambda x: abs(x[0]))[1]
            elif strategy == "right":
                scored.sort(key=lambda x: -x[0])         # most positive first
                best = scored[0][1]
                if scored[0][0] < 10:                     # no real right
                    best = min(scored, key=lambda x: abs(x[0]))[1]
            else:  # straight
                best = min(scored, key=lambda x: abs(x[0]))[1]
            nxt = best

        route.append(nxt)
        prev = node
        node = nxt

    return route


def route_centerline(network: RoadNetwork, route: list[str],
                     corner_radius_m: float = 6.0) -> list[tuple[float, float]]:
    """World-px centerline for a route, with junction corners rounded.

    The raw node-to-node line has sharp corners at every junction (up to
    ~100 deg) that no real car can follow at speed. We round them with the
    same circular-arc logic the road renderer uses (_round_polyline_corners)
    so the reference line is a smooth curve the car can actually track.
    """
    raw = [network.nodes[n] for n in route]
    radius_px = corner_radius_m * PPPM
    return _round_polyline_corners(raw, radius_px)


# ======================================================================
# Bicycle model
# ======================================================================

@dataclass
class Bicycle:
    x: float                 # world px
    y: float                 # world px
    heading: float           # deg, 0 = north
    v: float = 0.0           # m/s
    delta: float = 0.0       # steering angle, rad (+ = right)
    # --- car parameters (metres) ---
    wheelbase: float = 2.7
    max_steer: float = math.radians(38.0)   # ~5.2 m min turning radius
    accel: float = 2.8
    brake: float = 9.0
    drag: float = 0.02       # mild rolling/aero drag (1/s)
    a_lat_max: float = 3.5   # m/s^2 lateral-accel limit (understeer)

    def step(self, dt: float, throttle: float, brake: float, delta: float):
        # --- longitudinal ---
        if brake > 0:
            self.v -= self.brake * dt
        else:
            self.v += (self.accel * throttle - self.drag * self.v) * dt
        self.v = max(0.0, min(self.v, 40.0))   # cap ~144 km/h

        # --- steering (with understeer: cap heading rate by a_lat) ---
        delta = max(-self.max_steer, min(self.max_steer, delta))
        desired_dh = (self.v / self.wheelbase) * math.tan(delta)   # rad/s
        if self.v > 0.05:
            max_dh = self.a_lat_max / self.v                          # rad/s
            if abs(desired_dh) > max_dh:
                desired_dh = math.copysign(max_dh, desired_dh)
        else:
            desired_dh = 0.0
        self.heading = (self.heading + math.degrees(desired_dh) * dt) % 360.0

        # --- position ---
        h = math.radians(self.heading)
        self.x += self.v * math.sin(h) * dt * PPPM
        self.y += self.v * math.cos(h) * dt * PPPM


# ======================================================================
# Driver: speed profile (intent) + pure pursuit (control)
# ======================================================================

@dataclass
class Driver:
    cruise: float = 16.0          # m/s (~58 km/h)
    look_ahead: float = 8.0       # m base lookahead
    look_speed: float = 0.5       # lookahead grows with speed (m per m/s)
    profile_res: float = 1.0      # m, speed-profile sampling resolution
    # curvature -> speed law: v = sqrt(a_lat_max * R), with a safety factor
    corner_a_lat: float = 2.6     # m/s^2 we're willing to use in a corner

    def build_speed_profile(self, ref: RefLine) -> list[float]:
        """v_max at each profile sample (arc length i*profile_res).

        Forward pass: cap by what the curvature ahead allows.
        Backward pass: cap by what we can BRAKE to in time (a = brake).
        """
        n = int(math.ceil(ref.total / self.profile_res)) + 1
        v = [self.cruise] * n
        # curvature limit (forward): v <= sqrt(a_lat * R) = sqrt(a_lat / |k|)
        for i in range(n):
            s = i * self.profile_res
            if s > ref.total:
                break
            k = abs(ref.curvature_at(s))
            if k > 1e-4:
                v[i] = min(v[i], math.sqrt(self.corner_a_lat / k))
        # braking limit (backward): v[i] <= sqrt(v[i+1]^2 + 2*brake*ds)
        for i in range(n - 2, -1, -1):
            ds = self.profile_res
            v[i] = min(v[i], math.sqrt(max(0.0, v[i + 1] ** 2 + 2 * 9.0 * ds)))
        return v

    def target_speed(self, profile: list[float], s: float) -> float:
        i = int(s / self.profile_res)
        i = max(0, min(len(profile) - 1, i))
        return profile[i]

    def steer(self, car: Bicycle, ref: RefLine, s: float) -> float:
        """Pure pursuit: steer toward a point `lookahead` ahead on the
        reference line. Returns the steering angle (rad, + = right)."""
        lookahead = self.look_ahead + self.look_speed * car.v
        tx, ty = ref.point_at(s + lookahead)
        # vector from car to target, in the car's local frame
        dx = (tx - car.x) / PPPM
        dy = (ty - car.y) / PPPM
        h = math.radians(car.heading)
        # world -> car frame. Car forward = (sin h, cos h), right = (cos h, -sin h).
        local_right = dx * math.cos(h) - dy * math.sin(h)
        local_forward = dx * math.sin(h) + dy * math.cos(h)
        if local_forward < 0.5:
            local_forward = 0.5
        # desired steering so the front wheels point at the target
        delta = math.atan2(local_right, local_forward)
        return max(-car.max_steer, min(car.max_steer, delta))


# ======================================================================
# Simulation
# ======================================================================

@dataclass
class SimResult:
    frames: int
    distance_m: float
    offroad_frames: int
    max_offroad_m: float
    min_speed: float
    max_speed: float
    max_heading_delta_deg: float
    max_step_m: float
    positions: list = field(default_factory=list)   # (x, y, heading, v)
    ok: bool = False


def run_simulation(network: RoadNetwork, ref: RefLine, start: tuple[float, float, float],
                   driver: Driver, max_frames: int = 60 * 90,
                   record: bool = True) -> SimResult:
    x, y, heading = start
    car = Bicycle(x=x, y=y, heading=heading)
    profile = driver.build_speed_profile(ref)

    s = 0.0                      # arc length along the reference line
    offroad_frames = 0
    max_offroad_m = 0.0
    min_speed = float("inf")
    max_speed = 0.0
    max_heading_delta = 0.0
    max_step_m = 0.0
    positions: list = []
    prev_pos = (x, y)
    prev_heading = heading

    for frame in range(max_frames):
        # --- project car onto the reference line to get s (local search) ---
        s = _project_s(ref, car.x, car.y, s)

        # --- intent: target speed from the profile ---
        v_target = driver.target_speed(profile, s)

        # --- control: throttle / brake to the target speed ---
        if car.v < v_target - 0.3:
            throttle, brake = 1.0, 0.0
        elif car.v > v_target + 0.3:
            throttle, brake = 0.0, 1.0
        else:
            throttle, brake = 0.0, 0.0

        # --- control: pure-pursuit steering ---
        delta = driver.steer(car, ref, s)

        # --- integrate ---
        car.step(DT, throttle, brake, delta)

        # --- bookkeeping ---
        step_m = math.hypot(car.x - prev_pos[0], car.y - prev_pos[1]) / PPPM
        max_step_m = max(max_step_m, step_m)
        hd = abs((car.heading - prev_heading + 180) % 360 - 180)
        max_heading_delta = max(max_heading_delta, hd)
        min_speed = min(min_speed, car.v)
        max_speed = max(max_speed, car.v)

        # --- on-road check (the paved polygon) ---
        if not network.is_on_road(car.x, car.y):
            offroad_frames += 1
            # distance to the paved area (how far off)
            from shapely.geometry import Point
            d = network.get_paved_polygon().distance(Point(car.x, car.y)) / PPPM
            max_offroad_m = max(max_offroad_m, d)

        if record:
            positions.append((car.x, car.y, car.heading, car.v))
        prev_pos = (car.x, car.y)
        prev_heading = car.heading

        # stop if we've driven off the end of the route
        if s >= ref.total - 1.0 and car.v < 0.5:
            break

    dist = sum(
        math.hypot(positions[i][0] - positions[i - 1][0],
                   positions[i][1] - positions[i - 1][1]) / PPPM
        for i in range(1, len(positions))
    ) if positions else 0.0

    ok = (offroad_frames == 0) and (max_heading_delta <= 30.0) \
        and (max_step_m <= max_speed * DT + 0.1 * DT + 0.05)
    return SimResult(
        frames=len(positions) if positions else 0,
        distance_m=dist,
        offroad_frames=offroad_frames,
        max_offroad_m=max_offroad_m,
        min_speed=0.0 if min_speed == float("inf") else min_speed,
        max_speed=max_speed,
        max_heading_delta_deg=max_heading_delta,
        max_step_m=max_step_m,
        positions=positions,
        ok=ok,
    )


def _project_s(ref: RefLine, x: float, y: float, s_hint: float) -> float:
    """Project a world point onto the reference line, returning arc length.

    Uses s_hint as a starting point and searches a local window (the car
    moves a little each frame), with a global fallback if it wanders far.
    """
    best_s = s_hint
    best_d2 = float("inf")
    # local window: +- 30 m around the hint
    lo = max(0.0, s_hint - 30.0)
    hi = min(ref.total, s_hint + 30.0)
    steps = 60
    for k in range(steps + 1):
        s = lo + (hi - lo) * k / steps
        px, py = ref.point_at(s)
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = s
    # if the local best is still far, do a coarse global search
    if best_d2 ** 0.5 > 25 * PPPM:
        for k in range(0, 200):
            s = ref.total * k / 200
            px, py = ref.point_at(s)
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = s
    return best_s


# ======================================================================
# Rendering (optional PNG of the route + trajectory)
# ======================================================================

def render_png(network: RoadNetwork, ref: RefLine, result: SimResult, path: str,
               start: tuple[float, float, float]):
    import pygame
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()

    # bounds from the route + trajectory + margin
    xs = [p[0] for p in ref.pts] + [p[0] for p in result.positions]
    ys = [p[1] for p in ref.pts] + [p[1] for p in result.positions]
    m = 40 * PPPM
    minx, maxx = min(xs) - m, max(xs) + m
    miny, maxy = min(ys) - m, max(ys) + m
    W = int(maxx - minx)
    H = int(maxy - miny)
    W = max(W, 2)
    H = max(H, 2)
    surf = pygame.Surface((W, H))
    surf.fill((30, 90, 30))

    def to_img(wx, wy):
        # world y grows north (up); image y grows down -> flip
        return int(wx - minx), int(H - (wy - miny))

    # reference line (dashed centerline)
    for i in range(0, len(ref.pts) - 1, 1):
        a = to_img(*ref.pts[i])
        bpt = ref.point_at(ref.cum[i] + ref.seglen[i] / 2)
        b = to_img(*bpt)
        pygame.draw.line(surf, (230, 230, 120), a, b, max(1, W // 400))

    # trajectory
    if result.positions:
        traj = [to_img(p[0], p[1]) for p in result.positions]
        if len(traj) > 1:
            pygame.draw.lines(surf, (255, 60, 60), False, traj, max(1, W // 500))
        # off-road points in orange
        for p in result.positions:
            if not network.is_on_road(p[0], p[1]):
                pygame.draw.circle(surf, (255, 160, 0), to_img(p[0], p[1]), max(2, W // 200))

    # start marker
    pygame.draw.circle(surf, (60, 200, 255), to_img(*start[:2]), max(3, W // 150))

    # pygame cannot reliably encode PNG on this platform (see main.py) -
    # convert to PIL and save that way.
    from PIL import Image
    raw = pygame.image.tostring(surf, "RGB")
    img = Image.frombytes("RGB", (W, H), raw)
    img.save(path, "PNG")
    print(f"  📷 Rendered to {path}  ({W}x{H})")


# ======================================================================
# Scenario selection
# ======================================================================

def pick_start(network: RoadNetwork, name: str | None, strategy: str) -> tuple[str, str, tuple[float, float, float]]:
    """Return (start_node, strategy, (x, y, heading)).

    If `name` is a named start point (synthetic maps), use it. Otherwise
    pick a deterministic node on the map.
    """
    if name and name in network.start_points:
        x, y, heading, seg, fwd = network.get_start_point(name)
        seg_obj = network.segments[seg]
        start_node = seg_obj.start_node if fwd else seg_obj.end_node
        # infer a strategy from the name for variety
        strat = strategy
        return start_node, strat, (x, y, heading)

    if name:
        # treat `name` as a node id
        if name in network.nodes:
            x, y = network.nodes[name]
            return name, strategy, (x, y, 0.0)
        raise SystemExit(f"Unknown start '{name}'. Named start points: "
                         f"{sorted(network.start_points.keys())}")

    # default: a degree-1 or low node so the route is unambiguous
    node = None
    for nid in sorted(network.nodes.keys()):
        if network.node_degree.get(nid, 0) == 1:
            node = nid
            break
    if node is None:
        node = sorted(network.nodes.keys())[0]
    x, y = network.nodes[node]
    return node, strategy, (x, y, 0.0)


def load_network(args) -> RoadNetwork:
    if args.map:
        print(f"Loading synthetic test map: '{args.map}'")
        return build_test_map(args.map)
    from src.osm_loader import fetch_osm_data
    print("Loading OSM data…")
    bb = config.BOUNDING_BOX
    data = fetch_osm_data(bb["north"], bb["south"], bb["west"], bb["east"])
    return RoadNetwork.from_osm_data(data, bb["north"], bb["south"], bb["west"], bb["east"])


def main():
    ap = argparse.ArgumentParser(description="Bicycle-model car prototype")
    ap.add_argument("--map", default=None, help="synthetic test map name (e.g. 'basic'); default = OSM")
    ap.add_argument("--start", default=None, help="named start point or node id")
    ap.add_argument("--strategy", default="straight", choices=["straight", "left", "right"])
    ap.add_argument("--cruise", type=float, default=16.0, help="cruise speed m/s (~58 km/h)")
    ap.add_argument("--frames", type=int, default=60 * 90, help="max frames")
    ap.add_argument("--render", default=None, help="write a PNG of the run to this path")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    network = load_network(args)
    start_node, strategy, start = pick_start(network, args.start, args.strategy)
    print(f"\nStart: node={start_node}  pos=({start[0]:.0f},{start[1]:.0f})  "
          f"heading={start[2]:.1f}°  strategy={strategy}")

    route = build_route(network, start_node, strategy)
    print(f"Route: {len(route)} nodes")
    ref = RefLine.from_points(route_centerline(network, route))
    print(f"Reference line: {ref.total:.0f} m (corners rounded)")
    # Align the car's initial heading with the reference line so it starts
    # tracking cleanly (a mismatch would make it swing off the line at once).
    start = (start[0], start[1], ref.heading_at(0.0))

    driver = Driver(cruise=args.cruise)
    print(f"Cruise: {args.cruise:.1f} m/s ({args.cruise * 3.6:.0f} km/h)")
    print("Running bicycle model…\n")

    result = run_simulation(network, ref, start, driver, args.frames, record=True)

    print(f"Frames:            {result.frames}")
    print(f"Distance:          {result.distance_m:.0f} m")
    print(f"Speed:             {result.min_speed * 3.6:.0f}–{result.max_speed * 3.6:.0f} km/h")
    print(f"Max heading Δ:     {result.max_heading_delta_deg:.1f}°/frame")
    print(f"Max step:          {result.max_step_m:.3f} m/frame")
    print(f"Off-road frames:   {result.offroad_frames}")
    print(f"Max off-road dist: {result.max_offroad_m:.2f} m")
    print()
    if result.ok:
        print("✅ PASS: stayed on the road the whole run, no teleports/snaps.")
    else:
        print("❌ FAIL: went off-road and/or moved non-physically.")

    if args.render:
        render_png(network, ref, result, args.render, start)

    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())
