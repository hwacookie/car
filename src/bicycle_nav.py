# Bicycle-model road navigation
#
# Replaces the rail model (car._update_rails_mode + turning_system.py) for
# the "BICYCLE" driving mode. The car is a FREE PARTICLE on the road
# surface, not a train locked to the OSM graph:
#
#     state = (x, y, heading, v, steering_delta)
#     heading 0 = north (+y), forward = (sin h, cos h), positive steer = right
#
#     v          += (throttle - brake) * dt
#     d(heading) = (v / L) * tan(delta) * dt        (bicycle kinematics)
#     x         += v * sin(heading) * dt
#     y         += v * cos(heading) * dt
#
# Understeer is EMERGENT: the heading rate is capped by a lateral-accel
# limit (a_lat = v * |d(heading)/dt| <= A_LAT_MAX), so at speed the car
# cannot turn as sharply and swings wide instead of teleporting.
#
# Driving = two tiers (see docs/TURN_REWORK_PLAN.md):
#   intent  - a reference line (the route's rounded centerline, arc-length
#             parameterized) + a speed profile (v_max per arc-length point
#             from the curvature and a braking look-back constraint).
#   control - per frame: throttle/brake toward the profile, steer by pure
#             pursuit toward a lookahead point on the reference line.
#
# The ONLY inputs from the outside are the driver's high-level intent
# (accelerate / brake / which way to turn, from the blinkers) - exactly
# the same controls the rail model consumed, so the REST-API test
# framework (tests/test_turning.py) is unchanged.

from __future__ import annotations

import math

from . import config
from .road_network import _round_polyline_corners

PPPM = config.PIXELS_PER_METER


# ======================================================================
# Smoothed geometry (centripetal Catmull-Rom, arc-length parameterized)
# ======================================================================
# See docs/TURN_REWORK_PLAN.md §10: "the graph is sacred; the curve is a
# function of the graph." A road centerline is a centripetal Catmull-Rom
# spline through its original nodes (interpolating, C1, no kinks). Position
# / heading / curvature are evaluated on demand as a function of arc length
# s, via a dense arc-length lookup table (an evaluation CACHE of the math -
# it defines nothing new and touches no graph node).

def _cr_point(p0, p1, p2, p3, t0, t1, t2, t3, t):
    """One point on the centripetal Catmull-Rom piece between p1 and p2
    (given neighbors p0, p3 and knot values t0<t1<t2<t3, t in [t1, t2]).
    Barry-Goldman formulation; alpha=0.5 (centripetal) knot spacing is set
    by the caller, which is what prevents overshoot on irregular node
    spacing. Returns (x, y)."""
    a1x = (t1 - t) / (t1 - t0) * p0[0] + (t - t0) / (t1 - t0) * p1[0]
    a1y = (t1 - t) / (t1 - t0) * p0[1] + (t - t0) / (t1 - t0) * p1[1]
    a2x = (t2 - t) / (t2 - t1) * p1[0] + (t - t1) / (t2 - t1) * p2[0]
    a2y = (t2 - t) / (t2 - t1) * p1[1] + (t - t1) / (t2 - t1) * p2[1]
    a3x = (t3 - t) / (t3 - t2) * p2[0] + (t - t2) / (t3 - t2) * p3[0]
    a3y = (t3 - t) / (t3 - t2) * p2[1] + (t - t2) / (t3 - t2) * p3[1]
    b1x = (t2 - t) / (t2 - t0) * a1x + (t - t0) / (t2 - t0) * a2x
    b1y = (t2 - t) / (t2 - t0) * a1y + (t - t0) / (t2 - t0) * a2y
    b2x = (t3 - t) / (t3 - t1) * a2x + (t - t1) / (t3 - t1) * a3x
    b2y = (t3 - t) / (t3 - t1) * a2y + (t - t1) / (t3 - t1) * a3y
    cx = (t2 - t) / (t2 - t1) * b1x + (t - t1) / (t2 - t1) * b2x
    cy = (t2 - t) / (t2 - t1) * b1y + (t - t1) / (t2 - t1) * b2y
    return cx, cy


class SmoothCurve:
    """A centripetal Catmull-Rom spline through a list of 2D control points
    (world PIXELS), arc-length parameterized in METRES.

    Interpolating (the curve passes exactly through every control point -
    faithful to the OSM data) and C1-continuous (no kinks). `point_at`,
    `heading_at` and `curvature_at` are smooth functions of arc length s,
    evaluated against a dense arc-length lookup table built once in the
    constructor (a render/eval cache, not new geometry).

    heading: degrees, 0 = north (+y), positive = clockwise (east), matching
    the rest of the codebase (forward = (sin h, cos h)).
    curvature: 1/m, signed, positive = right turn (heading increasing).
    """

    def __init__(self, pts: list[tuple[float, float]], pppm: float = PPPM,
                 sample_m: float = 0.5):
        self.pppm = pppm
        self.pts = [(float(x), float(y)) for (x, y) in pts]
        n = len(self.pts)
        if n < 2:
            self.total = 0.0
            self._s = [0.0]
            self._x = [self.pts[0][0]] if n else [0.0]
            self._y = [self.pts[0][1]] if n else [0.0]
            self._hdg = [0.0]
            self._kap = [0.0]
            return

        # Centripetal knot values: alpha = 0.5.
        self._t = [0.0]
        for i in range(1, n):
            d = math.hypot(self.pts[i][0] - self.pts[i - 1][0],
                           self.pts[i][1] - self.pts[i - 1][1])
            self._t.append(self._t[-1] + max(d, 1e-6) ** 0.5)

        # Dense arc-length table. Sample each piece with a count proportional
        # to its chord length (tight/long pieces get more samples) so the
        # table is uniform in arc length to within ~sample_m.
        s_tab: list[float] = []
        x_tab: list[float] = []
        y_tab: list[float] = []
        hdg_tab: list[float] = []
        kap_tab: list[float] = []
        # Endpoint tangents: the curve leaves the first node toward the
        # second node and arrives at the last node from the second-to-last
        # (§10: "the spline ends at the junction node with a well-defined end
        # tangent, direction last-but-one -> last node"). We enforce this by
        # giving the duplicated endpoint knot a spacing that makes the
        # centripetal tangent at the endpoint equal to the adjacent-segment
        # direction (a standard clamped Catmull-Rom endpoint).
        cum = 0.0
        prev = None  # (x, y, hdg_rad) of the last emitted sample
        for i in range(n - 1):
            p1 = self.pts[i]
            p2 = self.pts[i + 1]
            t1 = self._t[i]
            t2 = self._t[i + 1]
            if i >= 1:
                p0 = self.pts[i - 1]; t0 = self._t[i - 1]
            else:
                # start: virtual p0 placed behind p1 along the p1->p2 direction
                p0 = p1; t0 = t1 - (t2 - t1)
            if i + 2 < n:
                p3 = self.pts[i + 2]; t3 = self._t[i + 2]
            else:
                # end: virtual p3 placed ahead of p2 along the p1->p2 direction
                p3 = p2; t3 = t2 + (t2 - t1)
            chord = math.hypot(p2[0] - p1[0], p2[1] - p1[1]) / pppm
            steps = max(8, int(chord / sample_m) + 1)
            for k in range(steps + 1):
                t = t1 + (t2 - t1) * k / steps
                x, y = _cr_point(p0, p1, p2, p3, t0, t1, t2, t3, t)
                if prev is not None:
                    ds = math.hypot(x - prev[0], y - prev[1]) / pppm
                    cum += ds
                    hdg = math.atan2(x - prev[0], y - prev[1])
                else:
                    hdg = math.atan2(p2[0] - p1[0], p2[1] - p1[1])
                first = (i == 0 and k == 0)
                last = (i == n - 2 and k == steps)
                if first:
                    s_tab.append(0.0); x_tab.append(x); y_tab.append(y)
                    hdg_tab.append(hdg); kap_tab.append(0.0)
                    prev = (x, y, hdg)
                    continue
                if last:
                    s_tab.append(cum); x_tab.append(x); y_tab.append(y)
                    hdg_tab.append(hdg); kap_tab.append(0.0)
                    break
                dhdg = (hdg - prev[2] + math.pi) % (2 * math.pi) - math.pi
                kap = dhdg / ds if ds > 1e-6 else 0.0
                s_tab.append(cum); x_tab.append(x); y_tab.append(y)
                hdg_tab.append(hdg); kap_tab.append(kap)
                prev = (x, y, hdg)

        self._s = s_tab
        self._x = x_tab
        self._y = y_tab
        self._hdg = hdg_tab
        self._kap = kap_tab
        self.total = s_tab[-1] if s_tab else 0.0

    def _index(self, s: float):
        """Binary search: largest i with self._s[i] <= s (clamped)."""
        lo, hi = 0, len(self._s) - 1
        if s <= self._s[0]:
            return 0
        if s >= self._s[hi]:
            return hi - 1 if hi > 0 else 0
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self._s[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def point_at(self, s: float) -> tuple[float, float]:
        if self.total <= 0:
            return self._x[0], self._y[0]
        s = max(0.0, min(self.total, s))
        i = self._index(s)
        s0, s1 = self._s[i], self._s[i + 1]
        f = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
        return (self._x[i] + f * (self._x[i + 1] - self._x[i]),
                self._y[i] + f * (self._y[i + 1] - self._y[i]))

    def heading_at(self, s: float) -> float:
        if self.total <= 0:
            return math.degrees(self._hdg[0])
        s = max(0.0, min(self.total, s))
        i = self._index(s)
        s0, s1 = self._s[i], self._s[i + 1]
        f = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
        h0, h1 = self._hdg[i], self._hdg[i + 1]
        dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
        return math.degrees(h0 + f * dh)

    def curvature_at(self, s: float) -> float:
        """Signed curvature (1/m) at arc length s (positive = right turn)."""
        if self.total <= 0:
            return 0.0
        s = max(0.0, min(self.total, s))
        i = self._index(s)
        s0, s1 = self._s[i], self._s[i + 1]
        f = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
        return self._kap[i] + f * (self._kap[i + 1] - self._kap[i])


class RefLine:
    """Arc-length-parameterized route centerline. Now a SmoothCurve
    (centripetal Catmull-Rom spline through the lane-offset points) instead
    of a raw chord polyline, so position/heading/curvature are smooth and
    kink-free (§10). The public interface (total, point_at, heading_at,
    curvature_at) is unchanged, so all callers work as before."""

    def __init__(self, pts: list[tuple[float, float]]):
        self._curve = SmoothCurve(pts, PPPM)
        self.total = self._curve.total
        self.pts = pts  # keep for backward compat / debugging

    def point_at(self, s: float) -> tuple[float, float]:
        return self._curve.point_at(s)

    def heading_at(self, s: float) -> float:
        return self._curve.heading_at(s)

    def curvature_at(self, s: float) -> float:
        return self._curve.curvature_at(s)


def _offset_polyline_right(pts: list[tuple[float, float]], offset_m: float) -> list[tuple[float, float]]:
    """Offset a polyline to the RIGHT by offset_m meters, perpendicular to
    the local driving direction (right-hand traffic / lane keeping).

    The direction at each point is the average of the incoming and outgoing
    segment unit-vectors, so the offset is smoothed across corners instead
    of jumping. The right vector for a tangent (dx, dy) in the north-up
    frame (heading 0 = north, forward = (sin h, cos h)) is (dy, -dx)
    normalized - i.e. the forward vector rotated 90 deg clockwise.
    """
    n = len(pts)
    if n < 2 or offset_m == 0:
        return list(pts)
    off_px = offset_m * PPPM
    out: list[tuple[float, float]] = []
    for i in range(n):
        if i == 0:
            dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        elif i == n - 1:
            dx, dy = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        else:
            dx_in, dy_in = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
            dx_out, dy_out = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            lin = math.hypot(dx_in, dy_in) or 1.0
            lout = math.hypot(dx_out, dy_out) or 1.0
            dx = dx_in / lin + dx_out / lout
            dy = dy_in / lin + dy_out / lout
        length = math.hypot(dx, dy) or 1.0
        rx, ry = dy / length, -dx / length  # right vector
        out.append((pts[i][0] + rx * off_px, pts[i][1] + ry * off_px))
    return out


def project_s(ref: RefLine, x: float, y: float, s_hint: float) -> float:
    """Project a world point onto the reference line -> arc length (m)."""
    best_s = s_hint
    best_d2 = float("inf")
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
    if best_d2 ** 0.5 > 25.0 * PPPM:
        for k in range(0, 200):
            s = ref.total * k / 200
            px, py = ref.point_at(s)
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = s
    return best_s


# ======================================================================
# Bicycle navigation
# ======================================================================

class BicycleNav:
    """Bicycle-model road following for one car.

    Consumes the driver's high-level intent (accelerate / brake /
    pending_turn) and integrates the bicycle kinematics, keeping the car
    on the road centerline and executing the signaled turn at the next
    junction.
    """

    WHEELBASE = 2.7            # m
    CAR_LENGTH_M = 4.5         # m (front bumper to rear bumper)
    MAX_STEER = math.radians(38.0)   # mechanical steering limit
    A_LAT_MAX = 2.0            # m/s^2 lateral-accel cap (understeer)
    A_CRUISE = config.CAR_ACCELERATION   # m/s^2 (2.8)
    A_BRAKE = config.CAR_BRAKING         # m/s^2 (10.0)
    V_MAX = config.CAR_SPEED             # m/s (50)
    # Standstill creep: a real car can roll forward slowly while steering
    # (release the brake, creep through a tight turn). Without this, a high
    # steering demand at spawn (e.g. a short approach into a sharp corner)
    # deadlocks: no speed -> no heading change (bicycle model) -> the
    # steering demand never drops -> the accel gate stays shut -> no speed.
    # Allow a minimum accel scale while the car is (nearly) stopped so it
    # can creep forward and break the deadlock. The creep is slow enough
    # that the lateral acceleration stays well under A_LAT_MAX.
    CREEP_SPEED = 1.0          # m/s - creep applies below this speed
    CREEP_SCALE = 0.3          # minimum accel scale while creeping
    # Corner-rounding radius for the reference line. Use the same radius as
    # the renderer's paved fillet (config.ROAD_CORNER_RADIUS_M = 6 m) so the
    # reference line follows the same corner the car is actually allowed to
    # occupy. A LARGER radius makes the reference arc bulge wider than the
    # paved fillet and the car clips the outside edge; a 6 m radius keeps the
    # car on the road through 90-degree corners, the hairpin, and the
    # roundabout (verified). (The car's understeer cap, not the radius, is
    # what protects it on tight bends - the speed profile slows it down.)
    CORNER_RADIUS_M = config.ROAD_CORNER_RADIUS_M  # 6.0
    HORIZON_SEGMENTS = 6     # how many segments ahead to build the route
    # Right-hand-traffic lane offset: the reference line is the road
    # CENTERLINE, but a car drives in its own lane (the right half of a
    # two-way road). Offset the reference line to the right by a quarter of
    # the road width (half the lane width) so the car keeps to its lane
    # instead of straddling the center. For the test map's 7 m two-way roads
    # this is 1.75 m. (On one-way roads the car would use the full width,
    # but the test map's one-way roads are short rings/spokes where a center
    # offset is close enough.)
    LANE_OFFSET_M = 1.75

    def __init__(self, car, network):
        self.car = car
        self.network = network
        self._ref: RefLine | None = None
        self._route: list[str] = []
        self._route_key: tuple | None = None
        self._route_seg_set: set[int] = set()
        self._profile: list[float] = []
        self._s = 0.0
        # Cruise at the car's top speed on straights: the car accelerates as
        # far as it can (limited by A_CRUISE / V_MAX) and only brakes when the
        # speed profile requires it (i.e. before a corner). This replaces the
        # old fixed 57 km/h cruise, which capped the car well below what it
        # could do.
        self._cruise = self.V_MAX

    # ---- route / reference line ----

    def _intended_turn(self) -> str:
        d = self.car.driver
        if d is not None and hasattr(d, "pending_turn"):
            return d.pending_turn or "straight"
        return "straight"

    def _build_route(self) -> list[str]:
        """Build a node route: the node BEHIND the car, then the current
        segment, then the signaled turn at the next junction, then straight
        on for a few segments.

        Starting at the behind-node (not the junction ahead) is essential:
        the reference line must extend forward from the car's own position,
        otherwise the car projects onto the line's start (s=0) and the
        braking profile - which propagates each corner's low speed backward
        along the line - would read as ~0 right where the car is, stalling
        it before it ever reaches the corner.
        """
        net = self.network
        car = self.car
        seg = net.segments[car.seg_idx]
        # The node we are heading TOWARD (the upcoming junction) and the
        # node BEHIND us (the route's start, so the line extends forward).
        junction = seg.end_node if car.forward else seg.start_node
        behind = seg.start_node if car.forward else seg.end_node
        route: list[str] = [behind]
        cur_seg = car.seg_idx
        cur_node = junction
        # First hop: the signaled turn at the upcoming junction.
        turn = self._intended_turn()
        for hop in range(self.HORIZON_SEGMENTS + 1):
            route.append(cur_node)
            if hop == 0:
                nxt = net.choose_next_segment(cur_seg, cur_node, turn)
            else:
                # After the first junction, normally go straight. But if we
                # are on a roundabout (one-way ring) and there is a
                # non-oneway exit that matches the intended turn, take it -
                # that is how the car leaves the roundabout.
                nxt = self._next_after_first(cur_seg, cur_node, turn)
            if nxt is None or nxt == cur_seg:
                # Dead end or no further road - stop extending.
                break
            nseg = net.segments[nxt]
            # The node on the far side of the next segment.
            cur_node = nseg.end_node if nseg.start_node == cur_node else nseg.start_node
            cur_seg = nxt
        return route

    def _next_after_first(self, cur_seg: int, cur_node: str, turn: str) -> int | None:
        """Segment to take at a junction AFTER the first one.

        Normally the straight-ahead continuation. On a roundabout (a
        one-way ring with two-way exit spokes) the car leaves at the first
        exit spoke it reaches going around the ring - the ring's one-way
        direction fixes the exit order, so the geometric turn direction of
        the exit (which depends on which way round you go) is not a
        reliable signal. This is what makes the car actually leave the
        roundabout instead of circling forever.
        """
        net = self.network
        candidates = []
        for idx in net.get_connected_segments(cur_node):
            if idx == cur_seg:
                continue
            angle = net.get_exit_angle(cur_seg, idx)
            candidates.append((idx, angle, net.segments[idx].oneway))
        if not candidates:
            return None
        # Roundabout exit: a non-oneway branch off a one-way ring. The
        # ring segments are oneway, so a two-way (or non-ring-oneway)
        # candidate is an exit spoke. Take the first one we reach.
        ring_oneway = any(ow for (_, _, ow) in candidates)
        if ring_oneway:
            exits = [(i, a) for (i, a, ow) in candidates if not ow]
            if exits:
                return min(exits, key=lambda x: abs(x[1]))[0]
        # Otherwise: straight continuation (smallest |angle|).
        return min(candidates, key=lambda x: abs(x[1]))[0]

    def _maybe_rebuild(self):
        """(Re)build the route + reference line when needed.

        The route must stay STABLE while the car follows it. In particular
        it must NOT be rebuilt every time the car crosses a node (changes
        seg_idx): rebuilding re-anchors the line at the new behind-node,
        which on a U-turn / hairpin drops the part of the line the car has
        already driven and makes the projected s jump, corrupting both the
        speed profile and the steering. We rebuild only when:
          * there is no line yet,
          * the intended turn changed (driver signaled a new turn), or
          * the car has reached the end of the line (extend ahead).
        """
        car = self.car
        turn = self._intended_turn()
        if self._ref is not None:
            if self._route_key == (car.seg_idx, turn):
                # Same segment + same intent: only extend if we've reached
                # the end of the line.
                if self._s < self._ref.total - 15.0:
                    return
            elif self._route_key is not None and \
                    self._route_key[1] == turn and \
                    car.seg_idx in self._route_seg_set:
                # We advanced to a segment that is part of the current
                # route (normal node crossing) with the same intent: keep
                # the line, just extend if near the end.
                if self._s < self._ref.total - 15.0:
                    return
        route = self._build_route()
        if len(route) < 2:
            # Degenerate (isolated node) - fall back to the current
            # segment's two endpoints so we still have a line to follow.
            seg = self.network.segments[car.seg_idx]
            route = [seg.start_node, seg.end_node]
        self._route = route
        raw = [self.network.nodes[n] for n in route]
        rounded = _round_polyline_corners(raw, self.CORNER_RADIUS_M * PPPM)
        # Shift the centerline into the driving lane (right-hand traffic).
        # The offset must be small enough that the car's right side stays on
        # the road for the NARROWEST segment in the route: the car's right
        # edge is at (offset + half_car_width) from the centerline, and the
        # road's right edge is at (width / 2). So offset <= width/2 - 0.9 -
        # margin. For a 7 m two-way road this is 1.75 m; for a 3.5 m one-way
        # road it's only 0.75 m (a fixed 1.75 m offset would push the car's
        # right wheels off a one-way road).
        min_width = min(
            (self.network.segments[i].width for i in self._route_segments()),
            default=7.0,
        )
        lane_offset = min(
            self.LANE_OFFSET_M,
            max(0.0, min_width / 2.0 - 0.9 - 0.1),
        )
        # For a LEFT turn, the right side of the car is on the OUTSIDE of the
        # turn. A full lane offset pushes the car to the outside of the turn,
        # which makes it swing wide. Reduce the offset for left turns so the
        # car stays closer to the centerline (and the inside of the turn).
        if turn == "left":
            lane_offset *= 0.5
        lane = _offset_polyline_right(rounded, lane_offset)
        self._ref = RefLine(lane)
        self._route_key = (car.seg_idx, turn)
        self._route_seg_set = self._route_segments()
        self._profile = self._build_speed_profile()
        # Re-project the car onto the (new) reference line.
        self._s = project_s(self._ref, car.x, car.y, self._s)

    def _route_segments(self) -> set[int]:
        """Segment indices covered by the current node route."""
        net = self.network
        idxs: set[int] = set()
        nodes = self._route
        for a, b in zip(nodes, nodes[1:]):
            for i, seg in enumerate(net.segments):
                if (seg.start_node == a and seg.end_node == b) or \
                        (seg.start_node == b and seg.end_node == a):
                    idxs.add(i)
                    break
        return idxs

    def _build_speed_profile(self) -> list[float]:
        """v_max at each 1 m arc-length point, from curvature + a braking
        look-back constraint.

        The constraint is FORWARD reachability: the speed at point i must
        be low enough that the car can still reach the (possibly lower)
        speed at point i+1 - i.e. it can brake from v[i] down to v[i+1]
        within the distance d. Equivalently, v[i] may be at most
        sqrt(v[i+1]^2 + 2*a_brake*d) (the speed from which a full brake
        over d still lands at v[i+1]). This is what creates the braking
        ramp INTO a corner. (The naive backward form sqrt(v[i+1]^2 -
        2*a*d) instead shrinks the limit on every straight metre and
        collapses to 0 - do not use it.)

        If the route ends at a dead end (no road beyond the last node),
        the car must come to a full stop there, so the terminal speed is
        0 and the braking ramp extends back from the end.
        """
        ref = self._ref
        n = max(2, int(ref.total) + 1)
        d = 1.0
        profile = [self._cruise] * n
        # Curvature cap: v = sqrt(a_lat_max / k), applied to the LOCAL
        # curvature at each point. (The old 60 m look-ahead max-curvature
        # cap is gone: it hard-capped the speed 60 m BEFORE a sharp fillet,
        # on perfectly straight road, and the braking ramp from cruise into
        # that cap was infeasible - e.g. a 90-degree fillet (R~6.3 m, cap
        # 3.55 m/s) forced 60 m of crawling before the corner even began.
        # The forward-reachability pass below already extends the corner
        # speed backward with a feasible braking ramp, so no lookahead is
        # needed for sharp corners. For chord-built curves (roundabout
        # ring) the local curvature reads 0 on chord middles and the true
        # ring curvature at the chord kinks, so the car runs a little
        # faster between kinks and brakes to the ring cap at each kink -
        # physically fine, and verified by the roundabout test.)
        for i in range(n):
            s = min(ref.total, i)
            k = abs(ref.curvature_at(s))
            if k > 1e-4:
                profile[i] = min(profile[i], math.sqrt(self.A_LAT_MAX / k))
        # Dead end: stop at the end of the route (no road beyond it).
        # The car's CENTER must stop a full car length before the end of
        # the pavement, otherwise the front bumper (and the front corners,
        # which the 4-corner on-road check uses) sticks out past the end
        # of the road. Set the whole tail to 0 so the braking ramp ends
        # at the stop line, not at the pavement edge.
        last_node = self._route[-1] if self._route else None
        if last_node is not None and self.network.node_degree.get(last_node, 0) <= 1:
            stop_idx = max(0, n - 1 - int(self.CAR_LENGTH_M))
            for i in range(stop_idx, n):
                profile[i] = 0.0
        # Forward reachability (braking) pass, from the end backward.
        for i in range(n - 2, -1, -1):
            v_next = profile[i + 1]
            v_reach = math.sqrt(v_next ** 2 + 2 * self.A_BRAKE * d)
            profile[i] = min(profile[i], v_reach)
        return profile

    def _target_speed(self, s: float) -> float:
        i = int(max(0.0, min(len(self._profile) - 1, s)))
        return self._profile[i]

    # ---- per-frame update ----

    def update(self, dt: float, control: dict):
        car = self.car
        self._maybe_rebuild()
        ref = self._ref
        if ref is None or ref.total < 1e-3:
            return

        # Project onto the reference line.
        self._s = project_s(ref, car.x, car.y, self._s)

        # --- steering: pure pursuit (computed BEFORE the longitudinal
        # step so the throttle can see the steering angle) ---
        # A SMALL base lookahead is critical for tight corners: if the
        # lookahead point lands beyond the corner (e.g. 10 m ahead on a 9 m
        # 90-degree arc), the car aims past the apex and cuts the corner
        # (swings wide). A 4 m base keeps the aim point inside the corner
        # at low speed; the 0.5*speed term still stretches it out on
        # straights for stability.
        lookahead = 4.0 + 0.5 * car.speed
        tx, ty = ref.point_at(self._s + lookahead)
        dx = (tx - car.x) / PPPM
        dy = (ty - car.y) / PPPM
        h = math.radians(car.heading)
        local_right = dx * math.cos(h) - dy * math.sin(h)
        local_forward = dx * math.sin(h) + dy * math.cos(h)
        if local_forward < 0.5:
            local_forward = 0.5
        delta = math.atan2(local_right, local_forward)
        delta = max(-self.MAX_STEER, min(self.MAX_STEER, delta))

        # --- longitudinal: throttle/brake toward the speed profile ---
        # The profile ALREADY encodes the cruising speed on straights and
        # the (much lower) corner speed at bends, with a braking ramp into
        # each corner. The accelerator means "go as fast as the profile
        # allows" - it must NOT override the corner limit (that is exactly
        # what made the car barrel into corners at cruise and swing wide).
        accel = control.get("accelerate", False)
        brake = control.get("brake", False)
        v_target = self._target_speed(self._s)
        if brake:
            v_target = 0.0
        # No flooring it mid-corner: after the apex the profile jumps back
        # to cruise speed while the car is still rotating (heading lags the
        # reference line). A real driver keeps the throttle off until the
        # wheels are nearly straight, so scale the acceleration rate by the
        # steering angle (full below ~5 deg, zero above ~25 deg).
        delta_deg = math.degrees(abs(delta))
        accel_scale = max(0.0, min(1.0, 1.0 - (delta_deg - 5.0) / 20.0))
        # Standstill creep (see CREEP_SPEED): let the car roll forward while
        # steering so a high steering demand at spawn can't deadlock it.
        if accel and car.speed < self.CREEP_SPEED:
            accel_scale = max(accel_scale, self.CREEP_SCALE)
        # Ease speed toward the target (accel / brake rates).
        if accel and car.speed < v_target:
            car.speed = min(v_target, car.speed + self.A_CRUISE * accel_scale * dt)
        elif car.speed > v_target:
            car.speed = max(v_target, car.speed - self.A_BRAKE * dt)
        car.speed = max(0.0, min(self.V_MAX, car.speed))
        car.target_speed = car.speed
        car._braking = brake or (car.speed > v_target + 0.05)

        # --- bicycle kinematics ---
        # Lateral-accel cap (understeer): limit the heading rate.
        max_rate = (self.A_LAT_MAX / car.speed) if car.speed > 0.3 else 0.0
        desired_rate = (car.speed / self.WHEELBASE) * math.tan(delta)
        rate = max(-max_rate, min(max_rate, desired_rate))
        car.heading = (car.heading + math.degrees(rate) * dt) % 360

        rad = math.radians(car.heading)
        car.x += math.sin(rad) * car.speed * dt * PPPM
        car.y += math.cos(rad) * car.speed * dt * PPPM

        # --- keep seg_idx / progress / forward in sync (for the API) ---
        self._sync_segment()

    def _sync_segment(self):
        """Re-derive seg_idx / progress / forward from the car's position
        so the REST-API state (segment, progress, forward) stays valid."""
        car = self.car
        net = self.network
        pppm = PPPM
        best_seg = car.seg_idx
        best_dist = float("inf")
        best_t = car.progress
        for idx, seg in enumerate(net.segments):
            dx = seg.x2 - seg.x1
            dy = seg.y2 - seg.y1
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                continue
            t = max(0.0, min(1.0, ((car.x - seg.x1) * dx + (car.y - seg.y1) * dy) / length_sq))
            proj_x = seg.x1 + t * dx
            proj_y = seg.y1 + t * dy
            dist = math.hypot(car.x - proj_x, car.y - proj_y)
            if dist < best_dist:
                best_dist = dist
                best_seg = idx
                best_t = t
        car.seg_idx = best_seg
        car.progress = best_t
        seg = net.segments[best_seg]
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_heading = math.degrees(math.atan2(dx, dy))
        car.forward = abs((car.heading - seg_heading + 180) % 360 - 180) < 90

    # ---- reset (after a teleport) ----

    def reset(self):
        self._ref = None
        self._route = []
        self._route_key = None
        self._profile = []
        self._s = 0.0
