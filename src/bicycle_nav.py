# Bicycle-model road navigation
#
# The car is a FREE PARTICLE on the road
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
from . import raceline

PPPM = config.PIXELS_PER_METER

# How much closer an off-route segment must be before it displaces the
# route we are actually following (see BicycleNav._sync_segment).
ROUTE_STICKINESS_PX = 3.0 * PPPM


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
    """An arc-length-parameterized polyline (the route's centerline).
    [TEMP-EXPERIMENT: reverted to 2066311 polyline to isolate the §10 spline]"""

    def __init__(self, pts: list[tuple[float, float]]):
        self.pts = pts
        self.seglen: list[float] = []
        self.cum: list[float] = [0.0]
        for i in range(len(pts) - 1):
            d = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) / PPPM
            self.seglen.append(d)
            self.cum.append(self.cum[-1] + d)
        self.total = self.cum[-1] if self.cum else 0.0

    def point_at(self, s: float) -> tuple[float, float]:
        s = max(0.0, min(self.total, s))
        for i in range(len(self.seglen)):
            if s <= self.cum[i + 1]:
                t = (s - self.cum[i]) / self.seglen[i] if self.seglen[i] > 0 else 0.0
                x = self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0])
                y = self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1])
                return x, y
        return self.pts[-1]

    def heading_at(self, s: float) -> float:
        s = max(0.0, min(self.total - 1e-6, s))
        for i in range(len(self.seglen)):
            if s <= self.cum[i + 1]:
                dx = self.pts[i + 1][0] - self.pts[i][0]
                dy = self.pts[i + 1][1] - self.pts[i][1]
                return math.degrees(math.atan2(dx, dy))
        dx = self.pts[-1][0] - self.pts[-2][0]
        dy = self.pts[-1][1] - self.pts[-2][1]
        return math.degrees(math.atan2(dx, dy))

    def curvature_at(self, s: float) -> float:
        """Signed curvature (1/m) at arc length s (positive = right turn)."""
        # Fixed physical window - see config.CURVATURE_WINDOW_M. A window
        # proportional to route length blurs short corners away entirely.
        h = config.CURVATURE_WINDOW_M
        s1 = max(0.0, s - h)
        s2 = min(self.total, s + h)
        if s2 - s1 < 1e-3:
            return 0.0
        h1 = math.radians(self.heading_at(s1))
        h2 = math.radians(self.heading_at(s2))
        dh = (h2 - h1 + math.pi) % (2 * math.pi) - math.pi
        return dh / (s2 - s1)


def _offset_polyline_right_varying(pts: list[tuple[float, float]],
                                   offsets_m: list[float]) -> list[tuple[float, float]]:
    """Offset a polyline to the right by a PER-POINT offset (metres).

    Same tangent averaging as _offset_polyline_right, but each point uses
    its own offset, so the lane position can vary smoothly along the line
    (e.g. drift to the outside of an upcoming turn only while approaching).
    """
    n = len(pts)
    if n < 2:
        return list(pts)
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
        off_px = offsets_m[i] * PPPM
        out.append((pts[i][0] + rx * off_px, pts[i][1] + ry * off_px))
    return out


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
    # Lateral-accel budget. Sets BOTH the cornering speed the profile
    # plans (v = sqrt(A_LAT_MAX / kappa)) and the understeer cap on the
    # heading rate, so the car is never handed a speed it cannot hold.
    # 2.0 was the plan's ultra-cautious "Miss Daisy" value; a real car
    # taking an ordinary local-road corner at ~17 km/h on a 6 m radius is
    # pulling 3.7 m/s^2 (0.38 g), so 2.0 made every corner about half the
    # speed a normal driver uses. Well under the ~8 m/s^2 the tyres could
    # actually give.
    A_LAT_MAX = 4.5            # m/s^2 lateral-accel cap (understeer)
    # The speed profile plans against only a FRACTION of that cap. Planning
    # at the full value means the heading rate is already saturated at the
    # apex, leaving the controller no authority to correct with: any
    # tracking error becomes permanent, and pure pursuit was measured
    # running ~1 m inside its own reference line through a bend - enough to
    # put the car's flank over the centreline. The reserve is what lets it
    # pull back onto the line.
    A_LAT_PLAN_FRACTION = 0.7
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
    CORNER_ARC_STEPS = 48      # see _maybe_rebuild: must out-resolve SAMPLE_M
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
    # Half-length of the span around a real junction where the wrong-side
    # check is suspended (no centreline exists inside an intersection).
    JUNCTION_SUPPRESS_M = 12.0
    # How far over the planned corner speed the car may be and still be
    # considered able to make the turn (the profile is sampled per metre,
    # so a little slack avoids rejecting turns on rounding alone).
    REACHABLE_SPEED_TOLERANCE = 1.15
    # When the driver signals a pull-over (right blinker + brake) and the
    # route ends at a dead end (the destination), the reference line drifts
    # to the right edge. The blend must FINISH before the car stops
    # (PARK_BLEND_END_M before the end) so the final stretch is a straight,
    # parallel line - otherwise the car stops at an angle (the offset line
    # is angled wherever the offset is still changing). The blend starts
    # PARK_BLEND_START_M before the end, while the car is still rolling in.
    PARK_BLEND_START_M = 40.0
    PARK_BLEND_END_M = 12.0
    # Pull-out: from right edge into normal lane (symmetric to park).
    PULL_OUT_START_M = 20.0
    PULL_OUT_END_M = 5.0
    # Lookahead (m) used to track the straight edge line in the final
    # pull-over stretch. Kept short so the car corrects its lateral offset
    # onto the edge quickly and holds the wheels parallel to the curb before
    # it stops (a long lookahead aims at the clamped endpoint and freezes
    # the car a few degrees off parallel, short of the edge).
    PARK_TRACK_LOOKAHEAD_M = 2.5
    # Lateral error (m) below which the car is considered "lined up" on the
    # edge line and the heading is squared up to parallel for the stop.
    PARK_ALIGN_LATERAL_M = 0.35

    def __init__(self, car, network):
        self.car = car
        self.network = network
        self._ref: RefLine | None = None
        self._route: list[str] = []
        self._route_key: tuple | None = None
        self._route_seg_set: set[int] = set()
        self._profile: list[float] = []
        self._s = 0.0
        self._pull_out_frames = 120  # ~2 seconds at 60fps, then done
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

    def _route_ends_dead_end(self) -> bool:
        """True if the current route ends at a dead end (the car's
        destination - there is no road beyond it)."""
        if not self._route:
            return False
        return self.network.node_degree.get(self._route[-1], 0) <= 1

    def distance_to_destination(self):
        """Distance (m) along the reference line to the end of the route,
        if the route ends at a dead end (the car's destination); else None."""
        if self._ref is None or not self._route_ends_dead_end():
            return None
        return max(0.0, self._ref.total - self._s)

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

    def _maybe_rebuild(self, pulling_over: bool = False, pulling_out: bool = False):
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
        key = (car.seg_idx, turn, pulling_over, pulling_out)
        if self._ref is not None:
            if self._route_key == key:
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
        # Round corners far more finely than the renderer does. The line is
        # resampled every 0.5 m and its curvature read over a 1 m window, so
        # the arc's own vertices must be much closer than that - at the
        # default 10 steps a 6 m fillet has ~0.94 m between vertices, and
        # each one reads as a 9-degree kink over a single 0.5 m sample:
        # curvature alternates between 0 and 0.31 (an apparent 3.3 m radius)
        # along an arc that is actually a clean 6 m.
        rounded = _round_polyline_corners(raw, self.CORNER_RADIUS_M * PPPM,
                                          arc_steps=self.CORNER_ARC_STEPS)
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
        max_offset = config.kerb_offset_m(min_width)
        base_offset = min(self.LANE_OFFSET_M, max_offset)
        # The driving line is the FASTEST LEGAL line, not a fixed lane
        # offset: minimum curvature inside the corridor bounded by the
        # pavement (never off-road) and the centreline (never on the
        # oncoming lane). See src/raceline.py. The old left/right
        # turn_offset constants are gone with it - they picked one lateral
        # position for the whole turn, in the wrong direction for right
        # turns, and had to be blended in and out, which is what made the
        # car S-wobble through bends.
        P, N, offsets, cum = raceline.solve_line(
            self.network, rounded, self._route_segments())
        lane = self._apply_end_blends(P, N, offsets, cum,
                                      edge_offset=max_offset,
                                      pulling_over=pulling_over,
                                      pulling_out=pulling_out)
        self._ref = RefLine(lane)
        self._route_key = key
        self._route_seg_set = self._route_segments()
        self._profile = self._build_speed_profile()
        # Re-project the car onto the (new) reference line.
        self._s = project_s(self._ref, car.x, car.y, self._s)

        # Is the signalled turn actually still REACHABLE from here?
        #
        # docs/TURN_REWORK_PLAN.md 2.5: the blinker means "turn this way at
        # the next decision point where it is still physically reachable",
        # not "turn here whatever happens". The speed profile already
        # encodes that: its braking pass guarantees the profile can be
        # followed from any point where the car is at or below it. So if we
        # are ALREADY faster than the profile allows at our own position,
        # no amount of braking gets us round the corner - the turn is out
        # of reach.
        #
        # Without this the route was rebuilt to take the turn regardless.
        # The car could not physically make it, understeered across the
        # junction and left the road - which is precisely the failure 2.5
        # describes, and it must instead SLIDE PAST and try again at the
        # next junction (the blinker stays on).
        if turn != "straight" and self._profile:
            v_allowed = self._target_speed(self._s)
            if car.speed > v_allowed * self.REACHABLE_SPEED_TOLERANCE:
                self._rebuild_straight_past(pulling_over, pulling_out)

    def _rebuild_straight_past(self, pulling_over: bool, pulling_out: bool):
        """Re-plan through the upcoming junction WITHOUT the signalled turn.

        The blinker deliberately stays on: the intent is not cancelled, it
        is deferred to the next junction where the turn is reachable.
        """
        car = self.car
        saved = self._route
        route = self._build_route_straight()
        if len(route) < 2 or route == saved:
            return
        self._route = route
        raw = [self.network.nodes[n] for n in route]
        rounded = _round_polyline_corners(raw, self.CORNER_RADIUS_M * PPPM,
                                          arc_steps=self.CORNER_ARC_STEPS)
        min_width = min(
            (self.network.segments[i].width for i in self._route_segments()),
            default=7.0,
        )
        P, N, offsets, cum = raceline.solve_line(
            self.network, rounded, self._route_segments())
        lane = self._apply_end_blends(P, N, offsets, cum,
                                      edge_offset=config.kerb_offset_m(min_width),
                                      pulling_over=pulling_over,
                                      pulling_out=pulling_out)
        self._ref = RefLine(lane)
        self._route_seg_set = self._route_segments()
        self._profile = self._build_speed_profile()
        self._s = project_s(self._ref, car.x, car.y, self._s)

    def _build_route_straight(self) -> list[str]:
        """_build_route(), but taking the straight continuation at the first
        junction instead of the signalled turn."""
        net = self.network
        car = self.car
        seg = net.segments[car.seg_idx]
        junction = seg.end_node if car.forward else seg.start_node
        behind = seg.start_node if car.forward else seg.end_node
        route = [behind]
        cur_seg, cur_node = car.seg_idx, junction
        for hop in range(self.HORIZON_SEGMENTS + 1):
            route.append(cur_node)
            nxt = (net.choose_next_segment(cur_seg, cur_node, "straight")
                   if hop == 0 else
                   self._next_after_first(cur_seg, cur_node, "straight"))
            if nxt is None or nxt == cur_seg:
                break
            nseg = net.segments[nxt]
            cur_node = nseg.end_node if nseg.start_node == cur_node else nseg.start_node
            cur_seg = nxt
        return route

    def _apply_end_blends(self, P: list[tuple[float, float]],
                          N: list[tuple[float, float]],
                          offsets: list[float],
                          cum: list[float],
                          edge_offset: float = 0.0,
                          pulling_over: bool = False,
                          pulling_out: bool = False) -> list[tuple[float, float]]:
        """Lay the manoeuvre blends over the racing line and build the line.

        `offsets` is the fastest legal line from raceline.solve_offsets().
        Two manoeuvres override it near the route's ends, because they are
        about where the car STOPS or STARTS rather than how fast it can
        get round a bend:

          pulling_over - drift to the kerb approaching the destination. The
              last PARK_BLEND_END_M are a CONSTANT offset so the car comes
              to rest parallel to the kerb, not at an angle.
          pulling_out  - the mirror image, leaving the kerb at the start.

        Also records where the route passes through real junctions, which
        is the only place LaneGuard has to be suppressed (see
        _in_turn_blend_zone).
        """
        offs = list(offsets)

        if pulling_over and edge_offset > 0:
            s_end = cum[-1]
            for i, s in enumerate(cum):
                d = s_end - s
                if d <= self.PARK_BLEND_END_M:
                    offs[i] = edge_offset
                elif d < self.PARK_BLEND_START_M:
                    t = (self.PARK_BLEND_START_M - d) / \
                        (self.PARK_BLEND_START_M - self.PARK_BLEND_END_M)
                    t = t * t * (3.0 - 2.0 * t)
                    offs[i] += (edge_offset - offs[i]) * t

        if pulling_out and edge_offset > 0:
            for i, s in enumerate(cum):
                if s <= self.PULL_OUT_END_M:
                    offs[i] = edge_offset
                elif s < self.PULL_OUT_START_M:
                    t = (s - self.PULL_OUT_END_M) / \
                        (self.PULL_OUT_START_M - self.PULL_OUT_END_M)
                    t = t * t * (3.0 - 2.0 * t)
                    offs[i] = edge_offset + (offs[i] - edge_offset) * t

        self._junction_zones = self._find_junction_zones(P, cum)
        return raceline.points_from_offsets(P, N, offs)

    def _find_junction_zones(self, rounded, cum) -> list[tuple[float, float]]:
        """Arc-length spans where the route crosses a real (degree >= 3)
        junction. Inside a junction there is no meaningful centreline to
        stay right of, so the wrong-side check cannot apply there. This
        used to be the whole route, which silently disabled the check
        everywhere."""
        zones = []
        for node in self._route:
            if self.network.node_degree.get(node, 0) < 3:
                continue
            nxy = self.network.nodes.get(node)
            if nxy is None:
                continue
            best_i, best_d2 = 0, float("inf")
            for i, (x, y) in enumerate(rounded):
                d2 = (x - nxy[0]) ** 2 + (y - nxy[1]) ** 2
                if d2 < best_d2:
                    best_d2, best_i = d2, i
            s = cum[best_i]
            zones.append((s - self.JUNCTION_SUPPRESS_M,
                          s + self.JUNCTION_SUPPRESS_M))
        return zones

    def _in_turn_blend_zone(self, s):
        """True where the wrong-side (LaneGuard) check must be suppressed.

        Only inside a real junction, where there is no centreline to stay
        right of. Previously this covered the ENTIRE route in both
        branches - a turning route because TURN_OFFSET_FAR_M (300 m)
        exceeded the route length, and a straight route because the
        default span was never overwritten - so the wrong-side check never
        ran at all and rule 2 was completely unenforced.
        """
        for a, b in getattr(self, "_junction_zones", ()):
            if a <= s <= b:
                return True
        return False

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
            # Take the WORST curvature within this metre, not the value at
            # its left edge. The profile is stored per metre but curvature
            # can peak sharply between samples (a junction fillet is only a
            # few metres long); sampling one point per metre stepped right
            # over those peaks and handed the car a corner speed its own
            # lateral limit forbids, so it understeered wide - off its line
            # and across the centreline.
            k = max(abs(ref.curvature_at(min(ref.total, i + f * 0.25)))
                    for f in range(4))
            if k > 1e-4:
                # v = sqrt(a_lat / k), and nothing else. There used to be
                # two extra clamps for k > 0.05 (a x0.4 penalty on A_LAT_MAX
                # and a hard 1.2 m/s ceiling) added to suppress
                # corner-cutting. They took a 6 m fillet from 12.5 km/h down
                # to 4.3 km/h - about a quarter of the ~17 km/h a real car
                # takes an ordinary local-road corner at (0.38 g). They
                # treated the symptom: the car cut corners because of where
                # its reference line runs, not because the speed formula was
                # wrong, so capping the speed just made it crawl and cut
                # corners slowly.
                profile[i] = min(profile[i], math.sqrt(
                    self.A_LAT_MAX * self.A_LAT_PLAN_FRACTION / k))
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
        # Pull-over mode: the driver is braking with the right blinker on
        # and the route ends at a dead end (the destination). In this mode
        # the speed profile (not a hard brake) controls the deceleration,
        # and the reference line is shifted to the right edge near the stop.
        driver = car.driver
        pulling_over = (
            control.get("brake", False)
            and driver is not None
            and getattr(driver, "blinker_right", False)
            and self._route_ends_dead_end()
        )
        # Pull-out mode: first ~2 seconds after spawn → max left steer + slow speed
        pulling_out = (self._pull_out_frames > 0)
        if pulling_out:
            self._pull_out_frames -= 1
        self._maybe_rebuild(pulling_over=pulling_over, pulling_out=pulling_out)
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
        # (swings wide). A 2.5 m base keeps the aim point inside the corner
        # at low speed; the 0.2*speed term still stretches it out on
        # straights for stability.
        lookahead = 2.0 + 0.15 * car.speed
        # Tighten lookahead further in high-curvature zones to prevent
        # corner-cutting on sharp junction fillets.
        local_k = abs(ref.curvature_at(self._s))
        if local_k > 0.05:
            lookahead = min(lookahead, 0.5)
        # Pull-over: track the reference line TIGHTLY (short lookahead) for
        # the whole drift-to-edge maneuver (the blend + the final straight).
        # The long default lookahead (4 + 0.5*speed, up to ~11 m at cruise)
        # makes the car cut the drift curve and stay well inside the edge,
        # and in the final straight it aims at the clamped endpoint (off the
        # approach path), freezing the heading a few degrees off parallel.
        # A tight aim point follows the curve to the edge and keeps the
        # wheels parallel to the curb, like a driver lining up before
        # pulling in.
        if pulling_over and (ref.total - self._s) < self.PARK_BLEND_START_M:
            lookahead = min(lookahead, self.PARK_TRACK_LOOKAHEAD_M)
        # Pull-out: very short lookahead for sharp left turn into lane.
        if pulling_out:
            lookahead = min(lookahead, 1.5)
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

        # No steering override while pulling out. The reference line
        # already starts at the kerb and merges into the lane (see
        # _apply_end_blends), so pure pursuit follows it correctly.
        # Forcing full left lock here instead ignored the line entirely and
        # drove the car straight across the centreline into the oncoming
        # lane - it only looked survivable because the wrong-side check was
        # suppressed for the whole route.

        # Pull-over final straight, once lined up: straighten the wheels to
        # the road so the car rests PARALLEL to the curb, not nose-in. The
        # tight tracking above has converged the car onto the edge line; once
        # its lateral error is small we stop chasing the (clamped) endpoint
        # and instead match the local line direction. This is a real turn -
        # the car is still rolling, and the lateral-accel clamp further down
        # keeps the implied turning radius above the physical minimum (no
        # in-place rotation, per the project's physics rule). Gating on the
        # lateral error (rather than distance) means the car first pulls to
        # the edge and only then squares up, so it ends both flush and level.
        if pulling_over and (ref.total - self._s) < self.PARK_BLEND_END_M:
            ref_x, ref_y = ref.point_at(self._s)
            lateral_err = math.hypot(car.x - ref_x, car.y - ref_y) / PPPM
            if lateral_err < self.PARK_ALIGN_LATERAL_M:
                line_heading = ref.heading_at(self._s)
                heading_err = (line_heading - car.heading + 180.0) % 360.0 - 180.0
                delta = max(-self.MAX_STEER, min(self.MAX_STEER, math.radians(heading_err)))

        # --- longitudinal: throttle/brake toward the speed profile ---
        # The profile ALREADY encodes the cruising speed on straights and
        # the (much lower) corner speed at bends, with a braking ramp into
        # each corner. The accelerator means "go as fast as the profile
        # allows" - it must NOT override the corner limit (that is exactly
        # what made the car barrel into corners at cruise and swing wide).
        accel = control.get("accelerate", False)
        brake = control.get("brake", False)
        v_target = self._target_speed(self._s)
        if brake and not pulling_over:
            v_target = 0.0
        # Pull-out: cap speed so curve is visible (~18 km/h, not 80).
        if pulling_out:
            v_target = min(v_target, 5.0)
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
        # Track the nearest segment that is on our ROUTE separately.
        route_seg, route_dist, route_t = None, float("inf"), car.progress
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
            if idx in self._route_seg_set and dist < route_dist:
                route_dist = dist
                route_seg = idx
                route_t = t
        # Prefer the route when it is a near-tie. At a fork the branches are
        # nearly equidistant, so plain nearest-segment can pick the one we
        # are NOT taking - and because leaving the route set triggers a
        # rebuild, that mis-assignment re-plans the car onto the wrong
        # branch and yanks the reference line out from under it. Measured on
        # the Y-junction: 1.3 m before the node the car flipped to the far
        # fork and was off the road 0.5 s later. We know which way we intend
        # to go; a couple of centimetres of projection noise should not
        # overrule it.
        if route_seg is not None and route_dist <= best_dist + ROUTE_STICKINESS_PX:
            best_seg, best_t = route_seg, route_t
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
        self._pull_out_frames = 120
