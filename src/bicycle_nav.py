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

import bisect
import math
import os

from . import config
from .road_network import _round_polyline_corners
from . import raceline

# Per-frame parking / route-cut tracing. Off by default: these fire on
# EVERY frame of a pull-over and drowned the console (and any test log)
# during the manoeuvre. Set CAR_DEBUG_PARK=1 to get them back.
PARK_DEBUG = bool(os.environ.get("CAR_DEBUG_PARK"))

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
        # Bisect over the monotone cumulative lengths (O(log n)): this is
        # called ~60x per car per substep from project_s, and the old
        # linear scan dominated multi-car frame time at hundreds of cars.
        i = max(0, min(bisect.bisect_right(self.cum, s) - 1,
                       len(self.seglen) - 1))
        t = (s - self.cum[i]) / self.seglen[i] if self.seglen[i] > 0 else 0.0
        return (self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0]),
                self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1]))

    def heading_at(self, s: float) -> float:
        s = max(0.0, min(self.total - 1e-6, s))
        i = max(0, min(bisect.bisect_right(self.cum, s) - 1,
                       len(self.seglen) - 1))
        dx = self.pts[i + 1][0] - self.pts[i][0]
        dy = self.pts[i + 1][1] - self.pts[i][1]
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


def _park_ease(t: float) -> float:
    """Lateral profile of the pull-over drift, t = 0 at the start of the
    swerve, 1 where the car is at the kerb offset.

    NOT a symmetric smoothstep. The binding constraint on how close to the
    kerb a car can park is its FRONT CORNER, which swings kerbwards
    whenever the body is slanted - and a symmetric profile puts its
    steepest slant exactly where the line is already halfway to the kerb,
    the worst possible place for it. This profile does the lateral work
    EARLY, while there is still road to spare, and arrives at the kerb
    almost parallel. Same zero slope at both ends (no kink for the
    controller), measurably closer parking: 0.47 m -> 0.35 m of gap on a
    7 m road with a 3 m drift.
    """
    return t * t * (3.0 - 2.0 * t)


def _park_ease_slope(t: float) -> float:
    """d/dt of _park_ease (per unit of normalised drift length)."""
    return 6.0 * t * (1.0 - t)


def _refine_project(ref: RefLine, x: float, y: float,
                    a: float, b: float) -> float:
    """Trisect [a, b] down to ~1 cm for the closest point on the line."""
    for _ in range(24):
        if b - a < 0.01:
            break
        m1 = a + (b - a) / 3.0
        m2 = b - (b - a) / 3.0
        p1 = ref.point_at(m1)
        p2 = ref.point_at(m2)
        d1 = (p1[0] - x) ** 2 + (p1[1] - y) ** 2
        dd2 = (p2[0] - x) ** 2 + (p2[1] - y) ** 2
        if d1 < dd2:
            b = m2
        else:
            a = m1
    return 0.5 * (a + b)


def project_s(ref: RefLine, x: float, y: float, s_hint: float,
              window: float = 30.0, global_fallback: bool = True,
              refine: bool = False) -> float:
    """Project a world point onto the reference line -> arc length (m).

    Only points within +/- window metres of s_hint are considered. On a
    plain route that is generous; on a folded line (U-turn) the window must
    be tight, because the spatially nearest point can belong to a DIFFERENT
    branch of the line than the one the car is actually driving on.

    NOTE on a tempting fast path (rejected): skipping the coarse scan when
    the hint point is "close" does NOT track continuous motion - _s would
    lag the car by up to the gate distance in a sawtooth until the gate
    trips, degrading pursuit precision. With bisect-based point_at the full
    61-point scan only costs ~10 us, so there is nothing left worth that
    risk.
    """
    best_s = s_hint
    best_d2 = float("inf")
    lo = max(0.0, s_hint - window)
    hi = min(ref.total, s_hint + window)
    steps = 60
    step = (hi - lo) / steps if steps else 0.0
    for k in range(steps + 1):
        s = lo + step * k
        px, py = ref.point_at(s)
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = s
    if global_fallback and best_d2 ** 0.5 > 25.0 * PPPM:
        step = ref.total / 200
        for k in range(0, 200):
            s = ref.total * k / 200
            px, py = ref.point_at(s)
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = s
    # Optional refinement to centimetres. The coarse scan resolves the line
    # only to ~1 m (30 m window / 60 steps), and that quantisation is not
    # harmless: for the parking plan a staircase in s becomes a staircase
    # in the brake demand (measured: the brake pulsing between 0 and A_PARK
    # every few frames, then a ~1 m overshoot past the stop point that
    # tripped the emergency full-brake); for steering it lets the pursuit
    # target land behind the car in tight corners. Refinement is OFF for
    # the steering projection on purpose - that is a behaviour change for
    # every manoeuvre on the map and is not part of this fix.
    if refine and step > 0.0:
        best_s = _refine_project(ref, x, y, max(0.0, best_s - step),
                                 min(ref.total, best_s + step))
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
    # Distance from the car's reference point (rear axle - what _s tracks)
    # to the front bumper. Used to park with the bumper AT the destination
    # flag rather than the axle on it (spec §1).
    FRONT_OVERHANG_M = (2.7 + config.CAR_LENGTH / 2.0
                        - config.FRONT_AXLE_OFFSET_M)
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
    # Speed the car arrives at any real junction (degree >= 3). Slow enough
    # that a turn signaled ANYWHERE inside the approach ramp is still
    # physically reachable: from 8 m/s to a ~4.5 m/s corner entry needs
    # only ~2 m of braking, so even a signal flicked right at the junction
    # mouth can be executed. Without this cap the car cruised up to V_MAX
    # into junctions - from 135 km/h a 90-degree corner needs ~70 m of
    # braking plus reaction time, while a human signals 20-60 m out, so
    # every turn was declared "out of reach" and the car slid past.
    JUNCTION_ENTRY_SPEED_M = 8.0
    # --- Brake & park plan (spec §1, user's reference table) ---
    # The approach to a destination is a STATELESS per-tick plan evaluated
    # from (distance to stop point, current speed):
    #   lead:   indicator on, hold speed          v * PARK_LEAD_S
    #   decel:  brake at A_PARK down to V_C       (v^2 - V_C^2) / (2 A_PARK)
    #   swerve: hold V_C, drift to the kerb       V_C * PARK_SWERVE_S
    #   final:  roll out to a stop, decel EASING OFF   V_C * PARK_STOP_TAU
    # No phase may decelerate faster than A_PARK - a real driver modulates
    # the pedal; full A_BRAKE stays reserved for emergencies.
    #
    # The final phase is NOT a constant-A_PARK brake. A constant
    # deceleration ends with the pedal still fully depressed at v = 0 -
    # the car snaps to a standstill (measured: -10 m/s^2 in the last
    # frames, because the constant-A target curve is unreachable in
    # discrete time and the car overshot the stop point). Spec §1 asks for
    # progressive braking instead: the target speed is proportional to the
    # remaining distance, v = d / PARK_STOP_TAU, so the deceleration
    # a = v / PARK_STOP_TAU decays with the speed and reaches zero exactly
    # at the stop point. PARK_STOP_TAU is chosen so the deceleration at
    # the START of the roll-out is exactly A_PARK: tau = V_C / A_PARK.
    A_PARK = config.PARK_BRAKING              # comfortable parking decel
    PARK_SWERVE_SPEED_M = config.PARK_CREEP_SPEED_M   # V_C, swerve speed
    PARK_LEAD_S = 2.0                          # indicator lead time at speed
    # Drift duration at V_C. It is a DISTANCE that matters (the line's
    # slant to the kerb is lateral shift / drift length), so this is sized
    # to keep the drift ~8 m long at the 2 m/s creep speed: measured, a 5 m
    # drift on a narrow street is steep enough that pursuit lags it and the
    # car ends up ~12 deg off parallel, with no speed left to square up.
    PARK_SWERVE_S = 1.5
    PARK_STOP_TAU = PARK_SWERVE_SPEED_M / config.PARK_BRAKING   # s
    # Extra road the CENTRELINE (fed into raceline.solve_line) keeps past
    # the destination, regardless of parking style. The corridor optimizer
    # needs road AFTER the last corner to resolve it properly - cutting the
    # corridor right at (or soon after) a corner starves it of that room
    # and it comes out clipped. Measured on the roundabout scenario: a
    # flag 75 m into the exit spoke, right after the ring-to-spoke corner,
    # produced a corner so pinched the car left the pavement mid-turn;
    # giving the corridor 30 m of run-out past the flag fixed it (0
    # violations) with no other change. This is a SOLVER INPUT ONLY - the
    # actual stop point (where the car parks) is unaffected, see
    # _dest_arc_s / _stop_margin.
    CORRIDOR_RUNOUT_M = 30.0
    # Below this speed the roll-out is over: the exponential approach never
    # reaches zero exactly, so the last 0.2 km/h are dropped. The implied
    # deceleration of that single step (0.05 m/s in one 60 Hz frame, 3
    # m/s^2) stays under A_PARK, so it is not a jerk.
    PARK_STANDSTILL_M_S = 0.02
    # Where the exponential roll-out hands over to a gentle constant
    # deceleration into the standstill (see update()).
    PARK_ROLL_END_M_S = 0.3
    PARK_ROLL_END_A = 0.6
    # Maximum slant of the pull-over drift (peak tangent angle). When the
    # car starts pulling over close to the dead end (e.g. right after a
    # U-turn that ends mid-road), the full kerb offset may be unreachable
    # without an extreme slide - at 27 deg of slant the front corner clips
    # the kerb even though the ideal line is geometrically clear, because
    # closed-loop pursuit cannot track such a steep drift exactly. Cap the
    # slant and park a bit further out instead (see _apply_end_blends).
    MAX_PARK_DRIFT_SLANT_DEG = 15.0
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
    # Swerve-zone geometry, DERIVED from the plan (not free parameters):
    # the line blends to the kerb over exactly the swerve phase (V_C held
    # for T_SWERVE), then holds a constant offset for the final brake so
    # the car comes to rest parallel to the kerb.
    # (with the progressive roll-out the final brake needs V_C * tau of
    # straight line, not V_C^2/(2 A_PARK) - it trades a longer, gentler
    # stop for the constant-decel one.)
    # ...and the straight-at-the-kerb stretch is longer than that roll-out
    # on purpose: spec §1 has a distinct "parallel ausrichten" phase, and
    # the controller needs it. Pursuit lags the drift by ~0.35 m, so a car
    # that is still drifting when it stops rests both short of the kerb and
    # a few degrees nose-in (measured: 5-12 deg). Creeping the last few
    # metres along a constant offset lets the lateral error and the heading
    # settle before the brake comes on.
    PARK_ALIGN_M = 3.0
    PARK_BLEND_END_M = max(PARK_ALIGN_M, PARK_SWERVE_SPEED_M * PARK_STOP_TAU)
    PARK_BLEND_START_M = PARK_SWERVE_SPEED_M * PARK_SWERVE_S \
        + PARK_BLEND_END_M                                        # ≈ 7.9 m
    # Pull-out: from right edge into normal lane (symmetric to park).
    PULL_OUT_START_M = 20.0
    PULL_OUT_END_M = 5.0
    # Lane change before parking (docs §1 variant on multi-lane roads):
    # a human in the overtaking lane changes lanes RIGHT first - well
    # before the spot - and only then parks; nobody backs in from the
    # left lane. A car starting IN THE PARKING LANE changes lanes LEFT
    # back into traffic: the parking lane is for parking, not for
    # travelling (user rules; replace the "hold initial line" rule of
    # 2026-08-27).
    # The lane change takes ~LANE_CHANGE_TIME_S (brisk, user rule) - so the
    # DISTANCE it covers is speed dependent: L = v * T (user rule), clamped.
    LANE_CHANGE_TIME_S = 2.5
    MERGE_LEN_MIN_M = 20.0        # crawl-speed floor for the blend length
    MERGE_LEN_MAX_M = 90.0        # high-speed cap
    MERGE_SETTLE_BEFORE_M = 40.0  # blend must end this far before the flag
    # The blinker comes on this far BEFORE the lane change starts (user
    # rule: always signal before changing lanes) and stays on until the
    # car has settled onto the new line.
    MERGE_SIGNAL_AHEAD_M = 30.0
    # Lookahead (m) used to track the straight edge line in the final
    # pull-over stretch. Kept short so the car corrects its lateral offset
    # onto the edge quickly and holds the wheels parallel to the curb before
    # it stops (a long lookahead aims at the clamped endpoint and freezes
    # the car a few degrees off parallel, short of the edge).
    PARK_TRACK_LOOKAHEAD_M = 2.5
    # Lateral error (m) below which the car is considered "lined up" on the
    # edge line and the heading is squared up to parallel for the stop.
    PARK_ALIGN_LATERAL_M = 0.35
    # Cross-track gain of the Stanley law used in that final straight
    # (see update()). 1.0 m/s of correction per metre of offset: at the
    # 2 m/s creep speed that is ~27 deg of steering for a half-metre of
    # error, and it decays smoothly to zero on the line.
    PARK_ALIGN_GAIN = 1.0
    # Heading-error gain of the same law. With the conventional 1:1 match,
    # the heading error decays with time constant WHEELBASE/v - at the
    # 2 m/s creep that is 1.35 s, but the alignment window (drift end to
    # roll-out) is only ~1 s, so the car mathematically can close barely
    # half its error and freezes nose-in (measured: +0.9 deg off-parallel,
    # wheels held hard over - see scripts/trace_park.py). x3 gives a 0.45 s
    # time constant at V_C: an 8 deg entry error is down to <1 deg before
    # the roll-out decay starts. The lateral-accel cap keeps the implied
    # radius physical (at 2 m/s and -15 deg that is ~10 m, no in-place
    # rotation).
    PARK_ALIGN_HDG_GAIN = 3.0
    # Cross-track fade of the same law (see update()): in the last metre
    # of the roll-out the lateral correction can no longer be completed -
    # at crawl speed it takes longer than the remaining distance, and while
    # it is being done the nose rotates PAST parallel (steering left to
    # pull a right-of-line car back onto the line). Measured on
    # corner_right_entry: the car froze 0.84 deg nose-in with 0.12 m of
    # cross error still uncorrected. A real driver straightens out and
    # rolls to a stop; the leftover lateral offset (a few cm) is invisible,
    # a crooked nose is not. The cross term fades to zero over
    # [PARK_ALIGN_CROSS_FADE_END_M, PARK_ALIGN_CROSS_FADE_START_M] of
    # remaining distance to the stop point.
    PARK_ALIGN_CROSS_FADE_START_M = 1.0
    PARK_ALIGN_CROSS_FADE_END_M = 0.4
    # Reserves kept when picking how close to the kerb to park: line
    # discretisation, and the controller's residual tracking error.
    PARK_LINE_MARGIN_M = 0.05
    PARK_TRACKING_MARGIN_M = 0.10

    # ---- Reverse-in parking (docs/DRIVING_MANEUVERS.md §1b) ----
    # Driving forwards, the body point that decides how close to the kerb a
    # car can get is its FRONT corner: it sits ~3.5 m ahead of the rear axle
    # (the point the reference line tracks), so a slant th throws it
    # 3.5*sin(th) kerbwards - more than the whole lateral move the swerve is
    # trying to make. Measured on a 7 m road: 0.47 m of gap left, with that
    # corner already sweeping to within 0.14 m of the kerb.
    # Reversing turns the geometry around: the REAR overhang is only ~0.9 m,
    # so backing in swings the short end into the space and the long end out
    # into the car's own lane. The car therefore pulls over forwards as
    # before and then tucks the last stretch in reverse.
    PARK_REVERSE_CREEP_M_S = 0.8        # ~3 km/h while reversing
    # The nose swings towards the centreline while backing in, and must stay
    # on our own half of the road: unlike a U-turn, parking may NOT use the
    # oncoming lane.
    PARK_CENTRELINE_MARGIN_M = 0.10
    # Reverse-in arc steering candidates, sharpest first (degrees). Full
    # lock is tried first because it is the shortest back-in; gentler locks
    # are only needed when the nose swing would otherwise cross into the
    # oncoming half - they let a deeper tuck stay legal (measured on the
    # 7 m two-way road: from lane offset 1.75 m, full lock caps the tuck
    # at ~0.3 m while 20 deg allows ~0.7 m over a ~4.5 m back-in).
    REVERSE_STEER_CANDIDATE_DEGS = (38.0, 30.0, 25.0, 20.0, 16.0)
    # Planning margin at the kerb for the swept body corners.
    PARK_REVERSE_KERB_MARGIN_M = 0.05
    # Below this the manoeuvre is not worth it - park forwards.
    PARK_REVERSE_MIN_TUCK_M = 0.10
    # Gains of the reverse-in follower (heading, and cross-track -> heading).
    # Feedback parking law gains (see _update_reverse_park): steer from
    # (heading error, offset error) only - no precomputed arc steering.
    # Path-following with corrections oscillated on this tight manoeuvre
    # (measured: eR swung +0.34 m outboard in arc 1 to -0.30 m inboard by
    # the end of the line; gain increases only moved the limit cycle).
    #   delta = PSI_GAIN * psi - POS_GAIN * e_off      (radians)
    # Reversing kinematics give the heading term a POSITIVE sign (a right
    # steer rotates the nose away from the kerb, damping a nose-towards-
    # kerb angle) and the position term a negative one (too far inboard ->
    # nose towards centreline swings the rear out to the kerb).
    PARK_REVERSE_PSI_GAIN = 2.8     # heading damping, ~1.2 s time constant
    PARK_REVERSE_POS_GAIN = 1.4     # offset correction, ~2.5 s at creep
    # Straight run-out at the end of the reverse arc (m). See
    # _start_reverse_park: the arcs alone leave the car still rotating.
    # Straight run-out after the arcs. Must be long enough for heading and
    # cross-track error to converge AT SPEED: below ~0.3 m/s the roll-out
    # decel engages and the car rotates proportionally slower, so a short
    # tail left the final straightening unfinished (measured: 6.5 deg
    # nose-in frozen for the last 0.3 m of the line).
    PARK_REVERSE_TAIL_M = 2.4
    # The staging stop can sit this much closer to the centreline than the
    # planned lane offset (the racing line is not exactly on LANE_OFFSET_M;
    # measured 1.55 vs 1.75 m on the basic map). A deeper start needs a
    # longer back-in for the same tuck, so plan from the deeper offset -
    # otherwise the executed tuck is stage-fit limited and the car parks
    # short of the kerb (measured: 0.42 m tuck instead of 0.89).
    PARK_PLAN_START_MARGIN_M = 0.35
    # ...but if the staging turned out shorter than planned, this much is
    # still enough to settle on.
    PARK_REVERSE_MIN_TAIL_M = 0.3

    # ---- U-turn (Wenden) - docs/DRIVING_MANEUVERS.md §5 ----
    # The maneuver is a reference line like any other: generated with the
    # SAME bicycle kinematics the car integrates, followed by the same pure
    # pursuit + speed-profile machinery. The only new ingredient is SIGNED
    # speed: the profile goes negative on the reverse step, and the pursuit
    # law mirrors (it aims the REAR of the car at the lookahead point).
    #
    # Stop angles were derived numerically (scripts/design_uturn.py) with
    # this car's geometry (R_rear = 3.46 m - tighter than a real car's, so
    # the spec's "120-150 deg" for step 3 does not fit a 7 m road; 90 deg
    # is the tightest feasible three-point recipe, verified with >= 0.15 m
    # corner clearance on roads from 7 m up). At full lock, no other angle
    # pair fits: larger th3 swings the rear past the near kerb, larger th2
    # swings the front past the far one.
    UTURN_SPEED_MAX = 2.8             # m/s (~10 km/h), spec: "5-10 km/h"
    UTURN_SINGLE_SWING_MIN_WIDTH_M = 11.0   # §5a vs §5b threshold (verified)
    UTURN_TH2_DEG = 60.0              # step-2 heading change before the stop
    UTURN_TH3_DEG = 90.0              # total heading change at end of step 3
    UTURN_STEP1_LEN_M = 14.0          # approach/brake stretch to the kerb
                                      # (the lateral blend length is derived
                                      # at generation time from the clearance
                                      # constraint - see _generate_uturn)
    # Planning allowance for the follower's tracking error: a line whose
    # body corners hug the pavement edge WILL be clipped in practice by a
    # pursuit controller with rate limits - plan like a driver keeps margin,
    # don't let the planned path touch the kerb.
    UTURN_BLEND_TRACKING_MARGIN_M = 0.30
    UTURN_TAIL_LEN_M = 15.0           # straight-out stretch after the turn
    UTURN_HOLD_S = 0.7                # pause at each full stop (spec: "Stoppen")
    UTURN_STALL_ABORT_S = 8.0         # no progress this long -> abort maneuver
    UTURN_MAX_ENTRY_SPEED_M = 5.0     # ~18 km/h - faster than this, a
                                      # three-point turn cannot be performed
    # Minimum corner clearance the generated LINE must guarantee against the
    # pavement edge. The hard rule applies to the CAR, and pure pursuit adds
    # a small tracking error on top of the line - so the line itself keeps a
    # margin (the design sweep in scripts/design_uturn.py shows this is the
    # binding constraint on 7 m roads: best achievable ~0.17 m).
    UTURN_LINE_MARGIN_M = 0.10

    def __init__(self, car, network):
        self.car = car
        self.network = network
        self._ref: RefLine | None = None
        self._route: list[str] = []
        self._route_key: tuple | None = None
        self._route_seg_set: set[int] = set()
        self._route_exit_dir: str | None = None
        self._route_exit_node: str | None = None
        self._turn_signal_target: str | None = None
        self._prev_pending_turn: str | None = None
        self._profile: list[float] = []
        self._s = 0.0
        # No pull-out on spawn: the car is placed in the normal driving
        # position (right-lane centre) by the spawner, not parked at the
        # kerb - the old curb-spawn + 2 s pull-out slowed every e2e test
        # down (decision 2026-08-27). The pull-out machinery below stays,
        # but is no longer triggered.
        self._pull_out_frames = 0
        # U-turn state (see _start_uturn / _update_uturn)
        self._uturn_active = False
        self._uturn_profile: list[float] = []   # SIGNED v_max per metre
        self._uturn_stops: list[float] = []     # arc length of full stops
        self._uturn_stop_ptr = 0
        self._uturn_state = 'drive'             # drive | creeping | holding
        self._uturn_hold_t = 0.0
        self._uturn_approach_dir = 0            # +1/-1: how we approach a stop
        self._uturn_mode = 'fwd'                # pursuit mode: fwd | rev
        self._uturn_stall_t = 0.0
        self._uturn_kappa: list[float] = []    # per-point curvature (rad/m)
        self._uturn_hdg: list[float] = []      # per-point nose heading (rad)
        self._uturn_lookahead_clamps: list[float] = []  # s values pursuit may not cross
        self._uturn_release_force = 0.0     # signed forced speed off a stop
        # Reverse-in parking (docs §1b): style of the current approach,
        # how much of the pull-over is left to the reverse tuck, and how far
        # ahead of the final spot the car stops first.
        # Lane-change episode (merge before parking): the zone [s0, s1] on
        # the current reference line, its direction, and the speed-dependent
        # blend length (frozen once committed - see _merge_params).
        self._merge_episode = None           # (s0, s1, 'left'|'right') or None
        self._merge_key = None               # episode identity (dest, spawn)
        self._merge_len = None               # frozen blend length for the episode
        # Blinker state for the driver: which indicator to show while a
        # lane change is pending/active (None = no lane-change signal).
        self.lane_change_signal = None       # 'left' | 'right' | None
        self._park_style = 'forward'        # forward | reverse
        self._park_style_locked = False     # decided once per destination
        self._park_tuck = 0.0
        self._park_stage_m = 0.0
        self._reverse_park = None           # active manoeuvre state, or None
        self._parked = False                # standing at the destination
        # Cruise at the car's top speed on straights: the car accelerates as
        # far as it can (limited by A_CRUISE / V_MAX) and only brakes when the
        # speed profile requires it (i.e. before a corner). This replaces the
        # old fixed 57 km/h cruise, which capped the car well below what it
        # could do.
        self._cruise = self.V_MAX
        # Explicit destination (world coords, e.g. the red end flag): the
        # reference line is truncated there and the car parks at it with the
        # same machinery as a dead-end route. None = no explicit destination.
        self._dest: tuple[float, float] | None = None

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

    def set_destination(self, x: float, y: float):
        """Set an explicit destination (world coordinates, e.g. the red end
        flag). The reference line is truncated there and the car parks at it
        with the same machinery as a dead-end route: parking ramp, drift to
        the kerb, stop one car length before the end (bumper at the
        destination).

        Forces a route rebuild on the next update: the destination can be
        set while the car is already following a line (the flag's segment
        only enters the route horizon as the car approaches - late on
        chord-heavy routes like roundabout rings), and node crossings alone
        do NOT rebuild (line stability). Without the forced rebuild the
        line would stay uncut and parking would never engage.
        """
        self._dest = (float(x), float(y))
        self._route_key = None   # invalidate -> _maybe_rebuild re-cuts
        self._park_style = 'forward'
        self._park_style_locked = False
        self._park_tuck = 0.0
        self._park_stage_m = 0.0
        self._park_stage_short = False
        self._reverse_park = None
        self._parked = False

    def clear_destination(self):
        """Remove an explicit destination (the red flag was cleared). The
        line must be rebuilt to extend past the old cut point again."""
        self._dest = None
        self._route_key = None
        self._park_style = 'forward'
        self._park_style_locked = False
        self._park_tuck = 0.0
        self._park_stage_m = 0.0
        self._reverse_park = None
        self._parked = False

    def _has_destination(self) -> bool:
        """True if the route has a place to stop: it ends at a dead end, or
        an explicit destination (red flag) was set."""
        return self._route_ends_dead_end() or self._dest is not None

    def distance_to_destination(self):
        """Distance (m) along the reference line to the destination (end of
        the route for dead-end routes, the explicit destination otherwise);
        None if the route has no destination."""
        if self._ref is None or not self._has_destination():
            return None
        return max(0.0, self._ref.total - self._park_s())

    def _lane_base_offset(self, max_offset: float) -> float:
        """Nominal lane offset for this run (m right of the centreline).

        The SETTLED line: the road's normal position - centre of the
        outermost DRIVING lane on multi-lane carriageways, fixed 1.75 m
        elsewhere (config.lane_base_offset_m) - clamped to what the route
        allows. If the car was SPAWNED at/right of that position (named
        start points with lateral_offset_m - the parking-lane scenario),
        the spawn line is the nominal one: the car holds it up to the flag
        and parks from it. Spawned LEFT of it, the car merges right onto
        this line first (see _merge_params) and parks from HERE - nobody
        backs in from the overtaking lane (user decision, replaces the
        "hold initial line" rule of 2026-08-27).
        """
        seg = self._dest_segment() or self.network.segments[self.car.seg_idx]
        base = min(config.lane_base_offset_m(seg.width, seg.lanes,
                                             seg.parking_lane_width,
                                             seg.oneway), max_offset)
        ovr = getattr(self.car, 'lane_offset_override_m', None)
        if ovr is not None:
            o_spawn = max(0.0, min(ovr, max_offset))
            # The parking lane is for PARKING, not for travelling (user
            # rule): a car spawned in it re-joins the driving lane first
            # and parks from there - the settled line stays at the normal
            # position, NOT the spawn line. Spawned inside the DRIVING
            # strip: hold the spawn line (docs §1 variant).
            drive_edge = seg.width / 2.0 - seg.parking_lane_width
            if not o_spawn > drive_edge + 0.25:
                base = max(base, o_spawn)
        return base

    def _merge_params(self, rounded, max_offset: float):
        """Lane-change blend for solve_line (docs §1 variant on multi-lane
        roads): (from_m, s0, s1) or None when the car holds its line.

        Applies to a spawned lateral offset OUTSIDE the normal driving lane
        with a flag destination ahead: LEFT of it (overtaking lane) the car
        changes lanes RIGHT onto the normal line; INSIDE THE PARKING LANE
        it changes lanes LEFT back into traffic - the parking lane is for
        parking, not for travelling (user rule). The blend runs over
        MERGE_BLEND_LEN_M and finishes MERGE_SETTLE_BEFORE_M before the
        flag, where the parking sequence (drive past + back-in) takes over
        from the settled line.
        """
        ovr = getattr(self.car, 'lane_offset_override_m', None)
        if ovr is None or self._dest is None:
            if PARK_DEBUG:
                print(f"[MERGE] none: ovr={ovr} dest={self._dest}")
            return None
        seg = self._dest_segment()
        if seg is None:
            if PARK_DEBUG:
                print("[MERGE] none: no dest segment")
            return None
        o_norm = min(config.lane_base_offset_m(seg.width, seg.lanes,
                                               seg.parking_lane_width,
                                               seg.oneway), max_offset)
        o_spawn = max(0.0, min(ovr, max_offset))
        drive_edge = seg.width / 2.0 - seg.parking_lane_width
        if not (o_spawn < o_norm - 0.25 or o_spawn > drive_edge + 0.25):
            self._merge_episode = None
            if PARK_DEBUG:
                print(f"[MERGE] hold: o_spawn={o_spawn:.2f} o_norm={o_norm:.2f} "
                      f"drive_edge={drive_edge:.2f}")
            return None                      # inside the driving strip: hold
        s_dest = self._dest_arc_s_on(rounded)
        if s_dest is None or s_dest < 30.0:
            self._merge_episode = None
            if PARK_DEBUG:
                print(f"[MERGE] none: s_dest={s_dest}")
            return None                      # not on this line / too close
        s1 = max(20.0, s_dest - self.MERGE_SETTLE_BEFORE_M)
        direction = 'right' if o_spawn < o_norm else 'left'
        # Speed-dependent blend length (user rule): the lane change takes
        # ~LANE_CHANGE_TIME_S, so L scales with the speed DURING the blend -
        # i.e. at zone entry, not at the end (sizing to the exit speed
        # stretches a 2.5 s change into a crawl when the car is still
        # accelerating in). The profile alone is the wrong estimate: it is
        # a CAP, not a prediction - from spawn the car can only build up
        # A_CRUISE per metre, so its ACTUAL entry speed is min(cap, built).
        # Iterate (s0 = s1 - L, v = entry(s0), L = v*T); both terms are
        # monotone in s0 before the braking ramp, so it settles fast.
        v_ref = self.car.speed
        if self._profile:
            last = len(self._profile) - 1
            L_g = self.MERGE_LEN_MIN_M
            for _ in range(3):
                s0_g = max(0.0, s1 - L_g)
                v_build = math.sqrt(
                    self.car.speed ** 2 +
                    2.0 * self.A_CRUISE * max(0.0, s0_g - self._s))
                v_entry = min(self._target_speed(min(s0_g, last)), v_build)
                L_g = min(self.MERGE_LEN_MAX_M,
                          max(self.MERGE_LEN_MIN_M,
                              v_entry * self.LANE_CHANGE_TIME_S))
            # The live speed guards against planning from far away (speed
            # ~0); the estimate is stable per route, so L is too.
            v_ref = max(v_ref, v_entry)
        key = (self._dest, round(o_spawn, 2))
        if getattr(self, '_merge_key', None) != key:
            self._merge_key = key
            self._merge_len = None           # new episode: recompute L
        L = self._merge_len
        if L is None:
            L = min(self.MERGE_LEN_MAX_M,
                    max(self.MERGE_LEN_MIN_M, v_ref * self.LANE_CHANGE_TIME_S))
            # Freeze once the car has committed to the change (at/near the
            # zone): re-planning the length under the wheels would step the
            # reference line laterally and pursuit would overshoot it.
            if s1 - self._s <= L + 15.0:
                self._merge_len = L
        s0 = max(0.0, s1 - L)
        if PARK_DEBUG:
            print(f"[MERGE] plan: o_spawn={o_spawn:.2f} o_norm={o_norm:.2f} "
                  f"s_dest={s_dest:.1f} s1={s1:.1f} v_ref={v_ref:.1f} "
                  f"L={L if L is None else round(L, 1)} s0={s0:.1f}")
        if s1 - s0 < 5.0:
            self._merge_episode = None
            return None
        self._merge_episode = (s0, s1, direction)
        return (o_spawn, s0, s1)

    @staticmethod
    def _merge_kwargs(merge) -> dict:
        """solve_line kwargs for a _merge_params result ({} to hold line)."""
        if merge is None:
            return {}
        o_spawn, s0, s1 = merge
        return dict(merge_from_m=o_spawn, merge_s0=s0, merge_s1=s1)

    def _dest_arc_s_on(self, pts) -> float | None:
        """Arc length (m) of the destination along `pts`, or None when it
        is not on this line (same 10 m guard as _cut_polyline_at)."""
        if self._dest is None or len(pts) < 2:
            return None
        x, y = self._dest
        best_i, best_t, best_d2 = 0, 0.0, float("inf")
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]
            vx, vy = pts[i + 1][0] - x0, pts[i + 1][1] - y0
            l2 = vx * vx + vy * vy
            if l2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - x0) * vx + (y - y0) * vy) / l2))
            d2 = ((x0 + t * vx - x) ** 2 + (y0 + t * vy - y) ** 2)
            if d2 < best_d2:
                best_i, best_t, best_d2 = i, t, d2
        if best_d2 > (10.0 * PPPM) ** 2:
            return None
        s = 0.0
        for i in range(best_i):
            s += math.hypot(pts[i + 1][0] - pts[i][0],
                            pts[i + 1][1] - pts[i][1])
        vx, vy = (pts[best_i + 1][0] - pts[best_i][0],
                  pts[best_i + 1][1] - pts[best_i][1])
        return (s + best_t * math.hypot(vx, vy)) / PPPM

    def _decide_park_style(self):
        """Forwards, or forwards-then-reverse (docs §1b)? Decided ONCE per
        approach, before the plan engages, because the answer changes the
        route line itself (it has to run past the parking spot) and the
        pull-over's target offset.

        Reversing needs three things: a real destination (a flag - you
        cannot drive past a dead end to back into it), road beyond it to
        stage the manoeuvre, and a tuck worth making.
        """
        if self._parked or self._reverse_park is not None:
            return
        if self._park_style_locked:
            return
        if self._dest is None or self._ref is None:
            if self._park_style != 'forward':
                self._park_style, self._park_tuck, self._park_stage_m = \
                    'forward', 0.0, 0.0
                self._route_key = None
            return
        if self._park_style == 'reverse':
            # Already committed. Only give up if the line turned out too
            # short to stage on (set by _cut_polyline_at) - and then LATCH
            # the choice. Re-deciding it every frame flip-flopped the style,
            # and since every decision invalidates the route key, the
            # reference line was rebuilt on EVERY frame of the approach: the
            # car was following a line that moved under it and left the road
            # (measured on the T-junction: 129 off-road frames).
            if getattr(self, '_park_stage_short', False):
                print("↩️  reverse-in parking dropped: not enough road "
                      "beyond the destination - parking forwards")
                self._park_style, self._park_tuck, self._park_stage_m = \
                    'forward', 0.0, 0.0
                self._park_stage_short = False
                self._park_style_locked = True
                self._route_key = None
            return
        # Not committed yet: decide while the destination is still far
        # enough away that changing the line costs nothing.
        d_stop = self.distance_to_destination()
        if d_stop is None or d_stop < self.PARK_BLEND_START_M + 5.0:
            return
        # The width that matters is the one AT THE DESTINATION, not under
        # the car: the decision is taken far upstream, and on the test map's
        # 'widths' street that meant planning a 1.7 m tuck from a 13 m
        # section for a spot on a 9 m one - a swing that would have put the
        # nose deep into the oncoming lane.
        width = self._dest_segment_width()
        # The forward position the tuck is planned FROM is where this car
        # actually drives (its nominal line - possibly a spawn offset).
        o_lane = self._lane_base_offset(config.kerb_offset_m(width))
        # Reverse-ONLY parking: the forward phase does NOT pull over. The
        # car drives straight past the spot in its lane and stops there;
        # the reverse covers the FULL lateral distance from lane to kerb -
        # the classic back-in. Planning the tuck from the drift target
        # instead left most of the parking to the forward swerve, so the
        # manoeuvre read as "forward park + a bit of reverse" (user
        # complaint 2026-08-26: show reverse parking only). The actual tuck
        # is re-solved from the car's real pose at staging anyway.
        o_fwd = o_lane
        plan = self.plan_reverse_tuck(width, o_fwd)
        if plan is None:
            return
        tuck, stage = plan
        self._park_style = 'reverse'
        self._park_tuck = tuck
        self._park_stage_m = stage
        self._park_stage_short = False
        self._route_key = None          # re-cut the line past the flag
        print(f"↩️  parking plan: reverse-in, {tuck:.2f} m tuck, "
              f"staging {stage:.2f} m past the flag "
              f"[car at ({self.car.x:.0f},{self.car.y:.0f}) "
              f"v={self.car.speed*3.6:.0f} km/h, in_turn="
              f"{self._in_turn_blend_zone(self._s)}]")

    def _dest_segment(self):
        """The road segment the destination sits on (None if no destination
        or it is not on the current route)."""
        if self._dest is None:
            return None
        dx_, dy_ = self._dest
        best, best_d2 = None, float('inf')
        for idx in (self._route_seg_set or set(range(len(self.network.segments)))):
            seg = self.network.segments[idx]
            ax, ay = seg.x2 - seg.x1, seg.y2 - seg.y1
            l2 = ax * ax + ay * ay
            if l2 < 1e-9:
                continue
            t = max(0.0, min(1.0, ((dx_ - seg.x1) * ax
                                   + (dy_ - seg.y1) * ay) / l2))
            px, py = seg.x1 + t * ax, seg.y1 + t * ay
            d2 = (px - dx_) ** 2 + (py - dy_) ** 2
            if d2 < best_d2:
                best_d2, best = d2, seg
        return best

    def _dest_segment_width(self) -> float:
        """Width (m) of the road segment the destination sits on."""
        seg = self._dest_segment()
        if seg is not None:
            return seg.width
        return self.network.segments[self.car.seg_idx].width

    def _stop_margin(self) -> float:
        """Distance (m) from the end of the reference line back to where
        the car's reference point (rear axle) comes to rest: the front
        overhang at a destination flag (bumper AT the flag, spec §1), a
        whole car length at a dead end (front corners stay on the road)."""
        return (self.FRONT_OVERHANG_M if self._dest is not None
                else self.CAR_LENGTH_M)

    def _park_s(self) -> float:
        """The car's arc position, refined to centimetres.

        The steering projection (`self._s`) is a 1 m-resolution scan and
        that is deliberately left alone, but the parking plan turns the
        distance-to-stop into a brake demand, so a metre of quantisation
        there becomes a pulsing brake pedal and a stop point missed by up
        to a metre. Refine locally around `self._s` for the plan only.
        """
        if self._ref is None:
            return 0.0
        return project_s(self._ref, self.car.x, self.car.y, self._s,
                         window=2.0, global_fallback=False, refine=True)

    def _parking_plan(self, d_stop: float, v: float):
        """Brake & park plan (spec §1). Stateless - evaluated every tick
        from the distance to the stop point and the current speed.

        Returns (phase, v_target) with phase in 'lead' | 'decel' |
        'swerve' | 'final', or None while d_stop > d_total(v) (cruise).
        v_target is the speed a plan-following car has at that point;
        'lead' carries None (keep following the normal profile), and
        'decel' never asks for MORE speed than the car already has.
        """
        vc = self.PARK_SWERVE_SPEED_M
        a = self.A_PARK
        # The longitudinal controller holds speed within a ±0.05 m/s
        # deadband of its target, so "reached V_C" must be tested with a
        # tolerance wider than that band - otherwise the car can rest at
        # V_C + 0.02 and never leave the decel phase.
        vc_tol = vc + 0.1
        d_decel = max(0.0, (v * v - vc * vc) / (2.0 * a))
        d_brake = d_decel + self.PARK_BLEND_START_M
        if v > vc_tol and d_stop <= d_brake:
            # Decel phase: brake at A_PARK; the plan-following speed at
            # distance d_stop reaches V_C exactly at the swerve start.
            curve = math.sqrt(vc * vc + 2.0 * a *
                              max(0.0, d_stop - self.PARK_BLEND_START_M))
            return ('decel', min(v, curve))
        # The roll-out zone is fixed by the CREEP speed, not by the current
        # one: sizing it as v*tau made the zone shrink as the car slowed,
        # so in the last centimetres the plan fell back out of 'final' into
        # 'swerve' (target V_C again) and back - the car sawed between 0.2
        # and 0.4 km/h at the kerb instead of coming to rest.
        d_final = max(v, vc) * self.PARK_STOP_TAU
        if d_stop <= d_final:
            # Final roll-out: target speed proportional to the remaining
            # distance, so the deceleration eases off to zero at the stop
            # point (spec §1: "kein Rucken beim Stillstand").
            return ('final', max(0.0, d_stop) / self.PARK_STOP_TAU)
        if d_stop <= self.PARK_BLEND_START_M:
            # Swerve zone: hold V_C (throttle up if approaching slower).
            return ('swerve', vc)
        if d_stop <= v * self.PARK_LEAD_S + d_brake:
            # Lead phase: indicator on, hold speed for T_LEAD seconds.
            return ('lead', None)
        return None

    def _cut_polyline_at(self, pts: list[tuple[float, float]],
                         x: float, y: float,
                         extend_m: float = 0.0) -> list[tuple[float, float]]:
        """Truncate `pts` at the point on the polyline closest to (x, y),
        keeping `extend_m` metres of road BEYOND it.

        The extension is what makes reverse-in parking possible: the car has
        to drive past its parking spot before it can back into it, so the
        reference line must not end at the flag.

        The destination lies on the route's CENTRELINE while the lane is
        offset from it (up to ~2.4 m), so the closest point on the polyline
        IS the arc-exact cut position. If the destination is not on this
        line at all (farther than the 10 m guard), `pts` is returned
        unchanged - a later rebuild, once the route covers it, will cut
        properly.
        """
        guard2 = (10.0 * PPPM) ** 2   # 10 m, in pixels
        best_i, best_t, best_d2 = 0, 0.0, float("inf")
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]
            vx, vy = pts[i + 1][0] - x0, pts[i + 1][1] - y0
            l2 = vx * vx + vy * vy
            if l2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - x0) * vx + (y - y0) * vy) / l2))
            px, py = x0 + t * vx, y0 + t * vy
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_i, best_t, best_d2 = i, t, d2
        # A two-point polyline is cuttable too: a single straight segment
        # arrives here as exactly two points, and refusing to cut it left
        # the centreline running on to the far dead end. The pull-over
        # drift was then built around THAT end and chopped off again when
        # the offset line was cut at the flag - so a car parking at a red
        # flag on a straight street never drifted to the kerb at all
        # (measured: it stopped 1.2 m from the kerb, mid-lane).
        if best_d2 > guard2 or len(pts) < 2:
            if PARK_DEBUG:
                print(f"[CUTDBG] NO CUT: closest={math.sqrt(best_d2)/PPPM:.1f} m "
                      f"(guard 10 m), n_pts={len(pts)}")
            return pts
        if PARK_DEBUG:
            print(f"[CUTDBG] cut: {len(pts)} -> {best_i + 2} pts, "
                  f"closest={math.sqrt(best_d2)/PPPM:.2f} m")
        cx = pts[best_i][0] + best_t * (pts[best_i + 1][0] - pts[best_i][0])
        cy = pts[best_i][1] + best_t * (pts[best_i + 1][1] - pts[best_i][1])
        out = list(pts[:best_i + 1]) + [(cx, cy)]
        if extend_m > 0.0:
            # Walk on along the original polyline from the cut point.
            left = extend_m * PPPM
            px, py = cx, cy
            j = best_i + 1
            while left > 1e-6 and j < len(pts):
                sx, sy = pts[j]
                d = math.hypot(sx - px, sy - py)
                if d <= 1e-9:
                    j += 1
                    continue
                if d >= left:
                    out.append((px + (sx - px) * left / d,
                                py + (sy - py) * left / d))
                    left = 0.0
                    break
                out.append((sx, sy))
                left -= d
                px, py = sx, sy
                j += 1
            if left > 1e-6:
                # Not enough road beyond the flag to stage the manoeuvre.
                self._park_stage_short = True
        return out

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
        # A roundabout exit that entered the horizon this build (node +
        # turn direction), so _maybe_rebuild can re-arm the driver's
        # signal for it - see there for why that matters.
        self._route_exit_dir: str | None = None
        self._route_exit_node: str | None = None
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
            # Leaving a one-way ring via a non-oneway spoke: remember the
            # exit's geometric direction (negative=left, positive=right).
            if hop > 0 and not nseg.oneway \
                    and net.segments[cur_seg].oneway:
                angle = net.get_exit_angle(cur_seg, nxt)
                self._route_exit_dir = 'right' if angle >= 0 else 'left'
                self._route_exit_node = cur_node
            # The node on the far side of the next segment.
            cur_node = nseg.end_node if nseg.start_node == cur_node else nseg.start_node
            cur_seg = nxt
        # Off-by-one: the loop's last hop RESOLVES one more segment (its
        # `nxt`) but never appends that segment's far node - so the final
        # decision in the horizon (e.g. a roundabout exit at the last ring
        # node) was known but not driven: the line ended at the junction
        # with no fillet, no corridor and no braking ramp for it. Measured
        # on the basic map's roundabout: the west exit entered the horizon
        # only when the car was ~20 m from its corner at 65 km/h - infeasible.
        if cur_node != route[-1]:
            route.append(cur_node)
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
                # the end of the line. Once close to a destination it never
                # needs extending any more - its end IS the stop point, and
                # re-solving the raceline every frame inside the last few
                # metres used to cost ~120 ms per frame (the whole parking
                # zone ran in slow motion).
                #
                # "Close" matters: freezing extension the MOMENT a distant
                # destination is set (rather than only near it) forces the
                # very first build to solve the raceline for the ENTIRE
                # route in one shot - every segment from here to the flag,
                # however many corners that is - instead of the normal
                # incremental ~HORIZON_SEGMENTS-at-a-time extension every
                # route uses. On a roundabout that one-shot route can chain
                # through the whole ring plus the exit spoke; the corridor
                # solve/corner-rounding over that many chained bends does
                # not have the same guarantees as the short, incremental
                # builds every other route gets, and it produced a
                # malformed line right at the ring-exit corner (measured:
                # the car spun through it and left the pavement). Only
                # freeze once actually near the flag, so everywhere else the
                # route builds and extends exactly like a normal one.
                near_dest = (self._dest is not None
                            and self._ref.total - self._s
                            < self.PARK_BLEND_START_M + 15.0)
                if near_dest:
                    return
                if self._s < self._ref.total - 15.0:
                    return
            elif (self._route_key is not None and \
                    self._route_key[1] == turn and
                    self._route_key[2] == pulling_over and
                    self._route_key[3] == pulling_out and
                    car.seg_idx in self._route_seg_set):
                # We advanced to a segment that is part of the current
                # route (normal node crossing) with the same intent: keep
                # the line, just extend if near the end.
                #
                # pulling_over/pulling_out must be part of this test: when
                # the driver starts braking for the dead end, the key CHANGES
                # even though nothing about the route did - and that change
                # must rebuild the line immediately (it is what adds the
                # drift-to-kerb blend). Swallowing it here used to delay the
                # pull-over until <15 m from the end, compressing the whole
                # drift into a couple of metres where no feasible target
                # offset exists - so the car hard-braked in the middle of
                # the lane and never pulled over.
                # (Only freeze extension near the destination - see above.)
                near_dest = (self._dest is not None
                            and self._ref.total - self._s
                            < self.PARK_BLEND_START_M + 15.0)
                if near_dest or self._s < self._ref.total - 15.0:
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
                                          arc_steps=self.CORNER_ARC_STEPS,
                                          fit_edges=True)
        if self._dest is not None:
            # Cut BEFORE solve_line: the pull-over drift blend in
            # _apply_end_blends anchors to the line's END, so the end must
            # already be the destination - cutting only afterwards would
            # leave the blend stranded on the chopped-off tail and the car
            # would never drift to the kerb.
            rounded = self._cut_polyline_at(rounded, *self._dest,
                                            extend_m=max(self._park_stage_m,
                                                        self.CORRIDOR_RUNOUT_M))
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
        base_offset = self._lane_base_offset(max_offset)
        # Normal driving settles each station at its OWN road's nominal
        # lane position, eased across road changes (raceline.auto_base).
        # A spawn lateral override pins the scalar line instead - the
        # parking scenarios must hold their initial line exactly.
        auto_base = getattr(self.car, 'lane_offset_override_m', None) is None
        # The driving line is the FASTEST LEGAL line, not a fixed lane
        # offset: minimum curvature inside the corridor bounded by the
        # pavement (never off-road) and the centreline (never on the
        # oncoming lane). See src/raceline.py. The old left/right
        # turn_offset constants are gone with it - they picked one lateral
        # position for the whole turn, in the wrong direction for right
        # turns, and had to be blended in and out, which is what made the
        # car S-wobble through bends.
        # Pass 1 - provisional line WITHOUT the merge blend: the speed
        # profile below must belong to THIS route, because the merge
        # length is speed dependent (user rule) and planning it against
        # the previous rebuild's profile read a dead-end braking ramp as
        # "entry speed" and stretched the change to the 90 m cap.
        P, N, offsets, cum = raceline.solve_line(
            self.network, rounded, self._route_segments(),
            base_offset=None if auto_base else base_offset,
            auto_base=auto_base)
        lane = self._apply_end_blends(P, N, offsets, cum,
                                      edge_offset=max_offset,
                                      pulling_over=pulling_over,
                                      pulling_out=pulling_out)
        if self._dest is not None:
            # Red-flag destination: the line ENDS there - parking ramp, kerb
            # drift and stop all anchor to the truncated end.
            lane = self._cut_polyline_at(lane, *self._dest,
                                         extend_m=self._park_stage_m)
        self._ref = RefLine(lane)
        if PARK_DEBUG:
            print(f"[REBUILD] key={key} old={self._route_key} "
                  f"car=({self.car.x:.0f},{self.car.y:.0f}) "
                  f"v={self.car.speed*3.6:.0f} phase={getattr(self,'park_phase','?')} "
                  f"merge={self._merge_episode}")
        # --- Turn-signal lifecycle -------------------------------------
        # Re-arm the signal for a roundabout exit that just entered the
        # horizon: after the entry turn is executed, pending_turn is
        # cleared (see below), but the exit still needs its intent -
        # without it _intended_turn() is 'straight', which disables the
        # junction approach cap at the exit node (gated on turn !=
        # 'straight'); measured: the car cruised the ring at 65 km/h and
        # arrived at the exit corner with ~20 m of braking distance.
        # Re-flicking for the exit is what a driver does too.
        d = car.driver
        if (self._route_exit_dir is not None and d is not None
                and hasattr(d, 'signal_turn')
                and getattr(d, 'pending_turn', None) is None):
            d.signal_turn(self._route_exit_dir)
            self._turn_signal_target = self._route_exit_node
            # Keep the key consistent with the re-armed intent, or every
            # following frame would see a "turn change" and rebuild.
            key = (car.seg_idx, self._route_exit_dir,
                   pulling_over, pulling_out)
        pending = getattr(d, 'pending_turn', None) if d is not None else None
        # A fresh signal (API/keyboard, not the re-arm above): remember
        # WHICH junction it is for - the one ahead of the car now.
        if (pending is not None and self._prev_pending_turn is None
                and self._turn_signal_target is None):
            seg = self.network.segments[car.seg_idx]
            self._turn_signal_target = (
                seg.end_node if car.forward else seg.start_node)
        # Auto-off: once the car has PASSED the junction the signal was
        # for, clear light + intent. (The old steering-cam version cleared
        # on any steering dip and killed multi-stage maneuvers mid-corner.
        # After a node crossing the rebuild above puts the passed node at
        # route[0] or drops it out of the route entirely.)
        if pending is not None:
            if (self._turn_signal_target is not None
                    and self._turn_signal_target not in self._route[1:]):
                d._clear_turn_signal()
                self._turn_signal_target = None
        else:
            self._turn_signal_target = None
        self._prev_pending_turn = (
            getattr(d, 'pending_turn', None) if d is not None else None)
        self._route_key = key
        self._route_seg_set = self._route_segments()
        self._profile = self._build_speed_profile()
        # Pass 2 - the merge blend, planned against the FRESH profile.
        # A lateral blend does not change a straight road's curvature, so
        # pass 1's profile stays valid for the re-solved line.
        merge = self._merge_params(rounded, max_offset)
        if merge is not None:
            P, N, offsets, cum = raceline.solve_line(
                self.network, rounded, self._route_segments(),
                base_offset=None if auto_base else base_offset,
                auto_base=auto_base, **self._merge_kwargs(merge))
            lane = self._apply_end_blends(P, N, offsets, cum,
                                          edge_offset=max_offset,
                                          pulling_over=pulling_over,
                                          pulling_out=pulling_out)
            if self._dest is not None:
                lane = self._cut_polyline_at(lane, *self._dest,
                                             extend_m=self._park_stage_m)
            self._ref = RefLine(lane)
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
                                          arc_steps=self.CORNER_ARC_STEPS,
                                          fit_edges=True)
        if self._dest is not None:
            # Cut before solve_line - see _maybe_rebuild: the pull-over
            # blend anchors to the line's end, which must be the stop point.
            rounded = self._cut_polyline_at(rounded, *self._dest,
                                            extend_m=max(self._park_stage_m,
                                                        self.CORRIDOR_RUNOUT_M))
        min_width = min(
            (self.network.segments[i].width for i in self._route_segments()),
            default=7.0,
        )
        max_offset = config.kerb_offset_m(min_width)
        merge = self._merge_params(rounded, max_offset)
        auto_base = getattr(self.car, 'lane_offset_override_m', None) is None
        P, N, offsets, cum = raceline.solve_line(
            self.network, rounded, self._route_segments(),
            base_offset=None if auto_base else self._lane_base_offset(max_offset),
            auto_base=auto_base,
            **self._merge_kwargs(merge))
        lane = self._apply_end_blends(P, N, offsets, cum,
                                      edge_offset=config.kerb_offset_m(min_width),
                                      pulling_over=pulling_over,
                                      pulling_out=pulling_out)
        if self._dest is not None:
            # Safety net: keep the endpoint exactly at the destination even
            # though the blend shifted the last metres laterally.
            lane = self._cut_polyline_at(lane, *self._dest,
                                         extend_m=self._park_stage_m)
            if PARK_DEBUG:
                print(f"[PARKDBG] ref.total={RefLine(lane).total:.1f} "
                      f"(pre-cut {cum[-1]:.1f})")
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

    def _dest_arc_s(self, P: list[tuple[float, float]],
                    cum: list[float]) -> float:
        """Arc length (m) of the point on P/cum closest to self._dest.

        Used instead of cum[-1] whenever the polyline may have extra
        run-out past the destination (see CORRIDOR_RUNOUT_M): cum[-1] is
        then the end of that run-out, not the stop point.
        """
        dx_, dy_ = self._dest
        best_i, best_d2 = 0, float("inf")
        for i_, (px_, py_) in enumerate(P):
            d2_ = (px_ - dx_) ** 2 + (py_ - dy_) ** 2
            if d2_ < best_d2:
                best_d2, best_i = d2_, i_
        return cum[best_i]

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

        # Reverse-ONLY parking (user decision 2026-08-26): a reverse-in
        # does NO forward pull-over drift at all - the line keeps its
        # normal lane offset to the staging stop, so the car drives past
        # the spot in its lane and the reverse covers the full lateral
        # distance to the kerb (the classic back-in).
        if pulling_over and edge_offset > 0 \
                and self._park_style != 'reverse':
            # Everything below is measured from the STOP POINT, not from
            # the line's end: the car halts a car length short of a dead
            # end (and its front overhang short of a flag), so anchoring
            # the drift geometry at the line end put the whole "align
            # parallel" stretch BEYOND the place where the car stops -
            # the car was still drifting when it came to rest, and ended
            # up nose-in and short of the kerb.
            #
            # When the CENTRELINE was given extra run-out past the flag
            # (see _maybe_rebuild / CORRIDOR_RUNOUT_M - the raceline solver
            # needs road AFTER the last corner to resolve it properly, or
            # it clips the corner: measured on the roundabout exit, the car
            # left the pavement mid-turn when the corridor ended right at
            # the corner), cum[-1] is NOT the stop point any more - it is
            # the end of that extra run-out. Anchor to the actual
            # destination position instead whenever there is one.
            if self._dest is not None:
                s_end = (self._dest_arc_s(P, cum) - self._stop_margin())
            else:
                s_end = cum[-1] - self._stop_margin()
            # Anchor the drift at the car's CURRENT position on this route.
            # Pulling-over activates when the driver starts braking for the
            # dead end - which can be well inside the nominal blend zone
            # (e.g. right after a U-turn, with the wall only ~20 m ahead).
            # Blending from the nominal start point would then step the line
            # laterally under the car; pursuit overshoots such a step and
            # clips the kerb. The drift always starts where the car is, at
            # its current offset.
            # The anchor is remembered once per pull-over episode: within
            # ~15 m of a dead end the route is rebuilt EVERY frame (line
            # extension), and re-anchoring at the moving car each time would
            # steepen the drift line continuously under the wheels.
            # The anchor is stored as a WORLD POINT, not as an arc length:
            # the route drops segments behind the car, so `s` is
            # re-zeroed while pulling over (measured: s_end jumping from
            # 100 m to 50 m mid-manoeuvre). Keying the anchor on s_end let
            # every one of those rebuilds re-anchor the drift at the moving
            # car, so the car chased a drift that kept being redrawn ahead
            # of it and came to rest ~1 m off its own line - off the
            # pavement, on the narrow street.
            def anchor_here():
                best_i, best_d2 = 0, float("inf")
                for i_, (px_, py_) in enumerate(P):
                    d2_ = (px_ - self.car.x) ** 2 + (py_ - self.car.y) ** 2
                    if d2_ < best_d2:
                        best_d2, best_i = d2_, i_
                s_ = cum[best_i]
                # P is in pixels, offsets are in metres - convert.
                o_ = ((self.car.x - P[best_i][0]) * N[best_i][0]
                      + (self.car.y - P[best_i][1]) * N[best_i][1]) / PPPM
                self._pull_over_anchor = (self.car.x, self.car.y, o_)
                return s_, o_

            anchor = getattr(self, "_pull_over_anchor", None)
            s_car = o_car = None
            if anchor is not None:
                ax_, ay_, o_a = anchor
                best_i, best_d2 = 0, float("inf")
                for i_, (px_, py_) in enumerate(P):
                    d2_ = (px_ - ax_) ** 2 + (py_ - ay_) ** 2
                    if d2_ < best_d2:
                        best_d2, best_i = d2_, i_
                # Usable only if the anchor point is still ON this line and
                # still far enough from the end to blend over.
                if (best_d2 <= (5.0 * PPPM) ** 2
                        and cum[best_i] <= s_end - self.PARK_BLEND_END_M - 1.0):
                    s_car, o_car = cum[best_i], o_a
            if s_car is None:
                s_car, o_car = anchor_here()
            # The blend covers the plan's swerve zone: the last
            # PARK_BLEND_START_M before the stop point. The car's own
            # position only anchors the drift when it is ALREADY inside
            # that zone (destination set very close, or a pull-over that
            # began late) - then starting anywhere else would step the
            # line laterally under the wheels.
            # While the car is still approaching, the drift must start
            # from the line's OWN offset at the zone entry. Using the
            # (possibly hundreds of metres old) anchored offset instead
            # put a lateral step into the line at the zone entry and the
            # car parked a full metre short of the kerb.
            if s_car <= s_end - self.PARK_BLEND_START_M:
                s_start = s_end - self.PARK_BLEND_START_M
                i_start = bisect.bisect_left(cum, s_start)
                i_start = max(0, min(len(offs) - 1, i_start))
                s_car, o_car = cum[i_start], offs[i_start]
            d_car = max(self.PARK_BLEND_END_M + 1.0,
                        min(self.PARK_BLEND_START_M, s_end - s_car))

            # Park as close to the kerb as the remaining distance allows:
            # the smoothstep drift's peak slant is 1.5*|dlat|/drift_len and
            # at mid-drift the front wheel contact patch reaches
            # off + WHEELBASE*sin(th) + TIRE_OUTBOARD*cos(th). Search for
            # the largest target offset that keeps every WHEEL on the
            # pavement - only the wheels need to stay on the road; the body
            # may overhang the kerb (a real pull-over swings the nose past
            # the curb line all the time). A driver who can't make it to
            # the kerb parks a bit further out, never at an angle into it.
            # Half the road, minus what the manoeuvre must keep in hand:
            # the U-turn's 0.30 m tracking reserve was far too pessimistic
            # here - with the Stanley alignment the car now ends within
            # 0.06 m of its own line, and every 0.1 m of unused reserve is
            # 0.1 m of gap to the kerb the driver can see.
            limit_lat = (edge_offset + config.CAR_WIDTH / 2.0
                         + config.KERB_CLEARANCE_M
                         - self.PARK_LINE_MARGIN_M
                         - self.PARK_TRACKING_MARGIN_M)
            # Wheel contact patch, not body corner: rear axle -> front
            # axle along the body, TIRE_OUTBOARD_M outboard (the paint
            # points). The nose overhang beyond the front axle may swing
            # past the kerb - it is in the air.
            front_reach = self.WHEELBASE
            lateral_reach = config.TIRE_OUTBOARD_M
            drift_len = d_car - self.PARK_BLEND_END_M

            def blend_offs(off_target):
                out = list(offs)
                for i_, s_ in enumerate(cum):
                    d_ = s_end - s_
                    if d_ <= self.PARK_BLEND_END_M:
                        out[i_] = off_target
                    elif d_ < d_car:
                        t_ = (d_car - d_) / drift_len
                        out[i_] = o_car + (off_target - o_car) * _park_ease(t_)
                return out

            def drift_worst(off_target):
                cand = blend_offs(off_target)
                worst = 0.0
                # ONLY the stretch the blend actually changes. Scanning the
                # whole line made the corner the car had just driven veto
                # the pull-over: through a left turn the racing line legally
                # swings wide, its "reach" exceeds the kerb limit, and since
                # that value does not depend on the candidate offset, EVERY
                # candidate was rejected and the car parked at whatever
                # offset it happened to have (measured on corner_left: no
                # drift at all, 1.07 m from the kerb, while the identical
                # right-hand corner drifted to 0.54 m). What happens outside
                # the drift zone is the raceline solver's business - it has
                # its own corridor.
                for i_ in range(1, len(P) - 1):
                    if cum[i_] < s_end - d_car:
                        continue
                    dx_ = (P[i_ + 1][0] + N[i_ + 1][0] * cand[i_ + 1]) - \
                          (P[i_ - 1][0] + N[i_ - 1][0] * cand[i_ - 1])
                    dy_ = (P[i_ + 1][1] + N[i_ + 1][1] * cand[i_ + 1]) - \
                          (P[i_ - 1][1] + N[i_ - 1][1] * cand[i_ - 1])
                    th_t = math.atan2(dx_, dy_)
                    th_a = math.atan2(P[i_ + 1][0] - P[i_ - 1][0],
                                      P[i_ + 1][1] - P[i_ - 1][1])
                    slant = abs(th_t - th_a)
                    reach = (cand[i_] + front_reach * math.sin(slant)
                             + lateral_reach * math.cos(slant))
                    worst = max(worst, reach)
                # ...plus an analytic scan of the drift profile itself. The
                # geometric tangent above under-reads the slant whenever the
                # drift is shorter than the polyline's sampling window, so
                # walk the profile directly: at each point the kerb-side
                # front corner reaches
                #     offset + front_reach*sin(th) + (W/2)*cos(th)
                # with th the local slant of the drift line.
                dlat_ = abs(off_target - o_car)
                if drift_len > 0.0:
                    n_ = 40
                    for k_ in range(n_ + 1):
                        t_ = k_ / n_
                        off_ = o_car + (off_target - o_car) * _park_ease(t_)
                        th_ = math.atan(dlat_ * _park_ease_slope(t_)
                                        / drift_len)
                        worst = max(worst,
                                    off_ + front_reach * math.sin(th_)
                                    + lateral_reach * math.cos(th_))
                return worst

            # Both feasibility constraints are monotone in the target
            # offset (bigger offset -> steeper slant AND bigger corner
            # reach), so a single downward search from the kerb finds the
            # best park position that satisfies both.
            max_slant = math.radians(self.MAX_PARK_DRIFT_SLANT_DEG)
            # Floor of the search: where the car already is - but never
            # further out than the kerb it is aiming for. On a road that
            # NARROWS towards the dead end the car's current offset can be
            # larger than the target kerb offset (measured on the 'widths'
            # street: pulling over at 1.85 m from the centreline of a 7 m
            # section, into a 4 m section whose kerb offset is 0.75 m).
            # Without the clamp the floor kept the line at 1.85 m and the
            # car parked off the pavement.
            o_target = min(max(o_car, 0.0), edge_offset)
            # The pull-over aims closer to the kerb than the driving line
            # is ever allowed to go: a parked car is not tracking anything.
            # When the last stretch is done in reverse (docs §1b), the
            # forward swerve stops short by exactly that tuck - driving
            # forwards it could not get there anyway without sweeping its
            # front corner over the kerb.
            park_edge = edge_offset + (config.KERB_CLEARANCE_M
                                       - config.PARK_KERB_CLEARANCE_M)
            cand_t = park_edge
            while cand_t > o_target + 1e-6:
                dlat_ = abs(cand_t - o_car)
                slant_ok = (drift_len <= 0.0) or \
                    (math.atan(1.5 * dlat_ / drift_len) <= max_slant)
                if slant_ok and drift_worst(cand_t) <= limit_lat:
                    o_target = cand_t
                    break
                cand_t -= 0.05
            # Remember how close the FORWARD swerve can actually get: the
            # reverse-in planner needs exactly that number - the tuck is
            # whatever is left over between it and the kerb.
            self._park_fwd_target = o_target
            offs = blend_offs(o_target)
            if PARK_DEBUG:
                print(f"[PARKDBG] s_end={s_end:.1f} s_car={s_car:.1f} "
                      f"o_car={o_car:.2f} edge={edge_offset:.2f} "
                      f"d_car={d_car:.2f} -> o_target={o_target:.2f}")

        # Forget the drift anchor only when the whole parking episode is
        # over. Clearing it on every rebuild that happens to run with
        # pulling_over=False (the plan's 'lead' phase, or a re-plan past a
        # junction) threw the anchor away mid-manoeuvre, and the next
        # rebuild re-anchored the drift at the moving car - the drift then
        # restarted under the wheels on every frame.
        if not pulling_over and getattr(self, 'park_phase', 'none') == 'none':
            self._pull_over_anchor = None

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

        If the route has a destination (dead end or explicit point, e.g.
        the red flag), the car must come to a full stop there with the
        documented parking approach (spec §1): comfortable A_PARK ramp,
        creep through the drift zone, soft release into the stop.
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
        # Junction approach cap: a braking ramp into every real junction
        # (degree >= 3) on this line, arriving at JUNCTION_ENTRY_SPEED_M.
        # See the constant for why turns are otherwise unreachable.
        #
        # Only while a turn is actually SIGNALLED. A real driver cruises
        # through junctions they intend to go straight past and brakes only
        # once they signal; capping EVERY junction unconditionally made the
        # car pulse accelerate-brake-accelbrate through dense town grids
        # (junctions every 30-60 m) - which reads as jerky, metronomic
        # driving. With the cap gated on the signal, cruising stays smooth;
        # signalling triggers a route rebuild that adds the ramp, and the
        # reachability check in _maybe_rebuild handles signals that come
        # too late (slide past, re-signal at the next junction).
        if self._intended_turn() != "straight":
            zones = getattr(self, "_junction_zones", ())
        else:
            zones = ()
        if zones:
            for i in range(n):
                s_i = i * d
                for (za, zb) in zones:          # sorted by s; first ahead wins
                    if za > s_i + 0.5:
                        profile[i] = min(profile[i], math.sqrt(
                            self.JUNCTION_ENTRY_SPEED_M ** 2
                            + 2 * self.A_BRAKE * (za - s_i)))
                        break
        # Parking ramp: when the route ends at a dead end, arrive at the
        # pull-over drift zone at PARK_ENTRY_SPEED_M, not at cruise speed.
        # Without this the only braking constraint is the stop line itself,
        # which from ~100 km/h means full A_BRAKE for the entire final
        # approach - the car slams to a halt in the middle of the lane with
        # no time left for the drift to the kerb. (Same shape as the
        # junction cap above; the backward reachability pass below blends
        # faster cars onto the ramp smoothly.)
        # Parking approach (spec §1): NOT shaped here - the stateless brake
        # & park plan (_parking_plan, evaluated in update()) owns the whole
        # destination approach: lead / decel at A_PARK / swerve at V_C /
        # final brake. The profile stays at cruise up to the plan's trigger
        # distance, which is exactly the plan's lead phase.
        # Forward reachability (braking) pass, from the end backward.
        for i in range(n - 2, -1, -1):
            v_next = profile[i + 1]
            v_reach = math.sqrt(v_next ** 2 + 2 * self.A_BRAKE * d)
            profile[i] = min(profile[i], v_reach)
        return profile

    def _target_speed(self, s: float) -> float:
        i = int(max(0.0, min(len(self._profile) - 1, s)))
        return self._profile[i]

    # ---- Reverse-in parking (docs/DRIVING_MANEUVERS.md §1b) ----

    @property
    def reverse_park_active(self) -> bool:
        return self._reverse_park is not None

    def _park_corner_offsets(self):
        """Body corner positions relative to the rear axle, (along, right)
        in metres, in the car's own frame."""
        front = self.WHEELBASE + (config.CAR_LENGTH / 2.0
                                  - config.FRONT_AXLE_OFFSET_M)
        rear = config.CAR_LENGTH / 2.0 - config.REAR_AXLE_OFFSET_M
        half = config.CAR_WIDTH / 2.0
        return [(front, half), (front, -half), (-rear, half), (-rear, -half)]

    def _reverse_park_path(self, v0: float, psi0: float, dv: float,
                           ds: float = 0.02, delta=None):
        """Two-arc reverse path in ROAD-LOCAL coordinates.

        (u = along the direction of travel, v = to the car's right, psi =
        heading relative to the road axis, + = nose towards the kerb.)
        The car reverses on `delta` lock one way, then `delta` the other,
        and ends parallel to the road (psi = 0) exactly dv further towards
        the kerb. Returns (points, ok) with points = [(u, v, psi, steer)]
        and u <= 0 throughout (the car moves backwards).

        Full lock is the SHORTEST back-in but also the sharpest nose swing;
        a gentler `delta` trades reverse length for a shallower swing - the
        same tuck keeps the front corners further off the oncoming half.

        Closed form for the swing angle: with R = 1/k the lateral gain of
        the pair of arcs is R*(cos(psi0) + 1 - 2*cos(psi0 - th)), so
            th = psi0 + acos((cos(psi0) + 1 - dv/R) / 2).
        """
        if delta is None:
            delta = self.MAX_STEER
        k = math.tan(delta) / self.WHEELBASE               # 1/R
        R = 1.0 / k
        arg = (math.cos(psi0) + 1.0 - dv / R) / 2.0
        if not -1.0 <= arg <= 1.0:
            return [], False
        th = psi0 + math.acos(arg)
        if th <= 1e-4:
            return [], False
        psi_min = psi0 - th
        pts = []
        u, v, psi = 0.0, v0, psi0
        pts.append((u, v, psi, self.MAX_STEER))
        # Arc 1: reversing with the wheels on full RIGHT lock swings the
        # rear towards the kerb and the nose away from it (signed-speed
        # bicycle kinematics: dpsi/dt = (v/L) tan(delta), v < 0).
        while psi > psi_min:
            psi -= k * ds
            u -= math.cos(psi) * ds
            v -= math.sin(psi) * ds
            pts.append((u, v, psi, self.MAX_STEER))
        # Arc 2: counter-lock, straightening back out to parallel.
        while psi < 0.0:
            psi += k * ds
            u -= math.cos(psi) * ds
            v -= math.sin(psi) * ds
            pts.append((u, v, psi, -self.MAX_STEER))
        return pts, True

    def _reverse_park_ok(self, pts, width: float) -> bool:
        """Every body corner of the whole manoeuvre stays on the pavement
        AND on our own half of the road (see PARK_CENTRELINE_MARGIN_M)."""
        kerb_lim = width / 2.0 - self.PARK_REVERSE_KERB_MARGIN_M
        centre_lim = -self.PARK_CENTRELINE_MARGIN_M
        corners = self._park_corner_offsets()
        for (_u, v, psi, _st) in pts:
            for (lf, lr) in corners:
                lat = v + lf * math.sin(psi) + lr * math.cos(psi)
                if lat > kerb_lim or lat < centre_lim:
                    return False
        return True

    def forward_drift_target(self, width: float, o_car: float) -> float:
        """How close to the kerb the FORWARD pull-over can get on a straight
        road of this width, starting from lane offset `o_car`.

        The same wheel-sweep rule _apply_end_blends applies to the real
        line (only the wheels must stay on the pavement), evaluated
        analytically so the parking style can be decided long before the
        drift is built.
        """
        drift_len = max(0.1, self.PARK_BLEND_START_M - self.PARK_BLEND_END_M)
        limit_lat = (width / 2.0 - self.PARK_LINE_MARGIN_M
                     - self.PARK_TRACKING_MARGIN_M)
        front_reach = self.WHEELBASE
        lateral_reach = config.TIRE_OUTBOARD_M
        cand = config.park_offset_m(width)
        while cand > o_car:
            dlat = abs(cand - o_car)
            worst = 0.0
            for k_ in range(41):
                t_ = k_ / 40.0
                off_ = o_car + (cand - o_car) * _park_ease(t_)
                th_ = math.atan(dlat * _park_ease_slope(t_) / drift_len)
                worst = max(worst,
                            off_ + front_reach * math.sin(th_)
                            + lateral_reach * math.cos(th_))
            if worst <= limit_lat:
                return cand
            cand -= 0.05
        return o_car

    def plan_reverse_tuck(self, width: float, o_forward: float):
        """How much of the pull-over is left for the reverse tuck.

        `o_forward` is how close to the centreline the FORWARD swerve can
        get on this road (the pull-over search's own answer). The tuck is
        the rest of the way to the kerb - reduced, if need be, until the
        swept body corners stay on the pavement and off the oncoming lane.

        Returns (tuck_m, stage_m): the lateral distance the reverse covers,
        and how far PAST the parking spot the car stops first. None when
        reversing buys nothing here.

        Searches over arc steering as well as tuck: a gentler lock gives a
        deeper feasible tuck (less nose swing into the oncoming half) at
        the price of a longer back-in, so the deepest park wins regardless
        of which lock makes it legal.
        """
        o_park = config.park_offset_m(width)
        start = max(0.0, o_forward - self.PARK_PLAN_START_MARGIN_M)
        best = None
        for delta_deg in self.REVERSE_STEER_CANDIDATE_DEGS:
            delta = math.radians(delta_deg)
            tuck = o_park - start
            while tuck >= self.PARK_REVERSE_MIN_TUCK_M - 1e-9:
                pts, ok = self._reverse_park_path(o_park - tuck, 0.0, tuck,
                                                  delta=delta)
                if ok and self._reverse_park_ok(pts, width):
                    # u is negative (backwards); add the straight run-out.
                    stage = -pts[-1][0] + self.PARK_REVERSE_TAIL_M
                    if best is None or tuck > best[0] + 1e-9:
                        best = (tuck, stage)
                    break
                tuck -= 0.05
        return best

    def _start_reverse_park(self) -> bool:
        """Generate and arm the reverse-in tuck from the car's ACTUAL pose.

        The car is standing at the staging point, roughly parallel, a little
        short of the kerb. Solve the two-arc path from where it really is
        (residual lateral and heading error included) to flush at the kerb;
        if the full depth is not reachable from here, take the deepest one
        that is - parking a few centimetres further out is fine, clipping
        the kerb or straddling the centreline is not.
        """
        car = self.car
        seg = self.network.segments[car.seg_idx]
        if car.forward:
            tx, ty = seg.x2 - seg.x1, seg.y2 - seg.y1
        else:
            tx, ty = seg.x1 - seg.x2, seg.y1 - seg.y2
        tl = math.hypot(tx, ty)
        if tl < 1e-6:
            return False
        tx, ty = tx / tl, ty / tl
        rx, ry = ty, -tx
        road_h = math.degrees(math.atan2(tx, ty)) % 360.0
        psi0 = math.radians((car.heading - road_h + 180.0) % 360.0 - 180.0)
        if abs(psi0) > math.radians(20.0):
            return False
        # Car position in the road frame (origin: the centreline point
        # abeam of the car).
        v0 = ((car.x - seg.x1) * rx + (car.y - seg.y1) * ry) / PPPM
        o_park = config.park_offset_m(seg.width)
        dv = o_park - v0
        if dv < self.PARK_REVERSE_MIN_TUCK_M:
            return False
        # How far back the car has to travel in total: exactly the distance
        # it was staged past its parking spot, so the front bumper ends up
        # on the flag again.
        stage = self._park_stage_m
        # Search tuck AND arc steering: from the full lane offset a deep
        # tuck is only legal with a gentler lock (less nose swing into the
        # oncoming half). The deepest feasible park wins; among equals the
        # sharpest (shortest) back-in is kept.
        best = None
        for delta_deg in self.REVERSE_STEER_CANDIDATE_DEGS:
            delta = math.radians(delta_deg)
            dv_try = o_park - v0
            while dv_try >= self.PARK_REVERSE_MIN_TUCK_M - 1e-9:
                cand, ok = self._reverse_park_path(v0, psi0, dv_try,
                                                   delta=delta)
                # The arcs must leave room for a straight run-out: the
                # follower arrives at the end of an arc still rotating
                # (feed-forward assumes perfect tracking), and without a
                # straight stretch to settle on, the car stopped 2.5 deg
                # nose-in.
                if (ok and self._reverse_park_ok(cand, seg.width)
                        and -cand[-1][0]
                        <= stage - self.PARK_REVERSE_MIN_TAIL_M):
                    if best is None or dv_try > best[0] + 1e-9:
                        best = (dv_try, cand)
                    break
                dv_try -= 0.05
        if best is not None:
            dv, pts = best
        else:
            pts = []
            # No feasible tuck from here after all. Still back up: the car
            # is standing PAST its destination (it drove there to stage the
            # manoeuvre), so reversing straight to the spot is the only way
            # to end up at the flag. It parks where the forward pull-over
            # left it laterally.
            pts = [(0.0, v0, psi0, 0.0)]
            dv = 0.0
            print("↩️  reverse tuck not feasible from here - backing "
                  "straight up to the destination")
        # Straight run-out: fills the remaining staging distance, and gives
        # heading and cross-track error somewhere to converge.
        u_end, v_end = pts[-1][0], pts[-1][1]
        tail = max(self.PARK_REVERSE_TAIL_M, stage + u_end)   # u_end < 0
        n_tail = max(1, int(tail / 0.02))
        for i_ in range(1, n_tail + 1):
            pts.append((u_end - i_ * tail / n_tail, v_end, 0.0, 0.0))
        # To world pixels. The path is anchored at the car's rear axle, so
        # the line starts exactly under the car - no lateral step to absorb.
        p0x, p0y = car.x, car.y
        line = [(p0x + (tx * u + rx * (v - v0)) * PPPM,
                 p0y + (ty * u + ry * (v - v0)) * PPPM)
                for (u, v, _psi, _st) in pts]
        ref = RefLine(line)
        self._ref = ref
        self._reverse_park = {
            'steer': [p[3] for p in pts],
            'psi': [p[2] for p in pts],
            'road_h': road_h,
            'v_target': o_park - (o_park - v0 - dv),   # reached kerb offset
            'o_park': o_park,
            'tx': tx, 'ty': ty, 'rx': rx, 'ry': ry,
            'seg': car.seg_idx,
        }
        self._s = 0.0
        print(f"\n↩️  REVERSE-IN parking: {dv:.2f} m tuck over "
              f"{ref.total:.2f} m of reverse, target {o_park:.2f} m from the "
              f"centreline\n")
        return True

    def _update_reverse_park(self, dt: float):
        """Per-frame execution of the reverse-in tuck (replaces the normal
        update body while it runs).

        Steering is a pure FEEDBACK parking law - steer from (heading
        error, offset error) only, the way a driver does it. The planned
        two-arc path is used for feasibility and staging distance, not for
        steering: following its full-lock feed-forward with correction
        loops oscillated on this tight manoeuvre (the corrections fight
        each other; measured residuals up to 0.3 m / 9 deg at the end of
        the line). The feedback law converges offset and heading
        simultaneously, so the car stops parallel AND at the kerb.

        State (road frame of the parking segment):
            e_off = rear-axle offset - o_park   (+ = too far towards kerb)
            psi   = nose relative to road axis  (+ = nose towards kerb)
        Reversing kinematics (v < 0): a right steer (delta > 0) rotates the
        nose AWAY from the kerb, which swings the rear TOWARDS it - hence
        delta = +PSI_GAIN*psi - POS_GAIN*e_off (heading damping is positive
        in reverse: the classic counter-intuitive bit).
        """
        car = self.car
        ref = self._ref
        st = self._reverse_park
        self._s = project_s(ref, car.x, car.y, self._s,
                            window=1.0, global_fallback=False, refine=True)
        d_rem = max(0.0, ref.total - self._s)

        # --- state ---
        psi_car = math.radians((car.heading - st['road_h'] + 180.0) % 360.0
                               - 180.0)
        rx_, ry_ = st['rx'], st['ry']
        seg = self.network.segments[st['seg']]
        v_car = (((car.x - seg.x1) * rx_ + (car.y - seg.y1) * ry_) / PPPM)
        e_off = v_car - st['o_park']
        converged = (abs(e_off) < 0.06
                     and abs(psi_car) < math.radians(1.5))

        # --- longitudinal: creep backwards; brake to a stop when the pose
        # has converged or the line is used up (progressive roll-out in the
        # last moment, same as the forward park).
        if converged or d_rem <= 0.05:
            v_target = 0.0
        else:
            v_target = -min(self.PARK_REVERSE_CREEP_M_S,
                            d_rem / self.PARK_STOP_TAU)
        brake_rate = self.A_PARK
        if abs(car.speed) < self.PARK_ROLL_END_M_S:
            brake_rate = min(brake_rate, self.PARK_ROLL_END_A)
        if car.speed > v_target + 0.02:
            car.speed = max(v_target, car.speed - brake_rate * dt)
        elif car.speed < v_target - 0.02:
            car.speed = min(v_target, car.speed + self.A_CRUISE * dt)
        car._braking = False
        car.target_speed = car.speed

        # --- steering: feedback parking law + envelope guard ---
        delta = (self.PARK_REVERSE_PSI_GAIN * psi_car
                 - self.PARK_REVERSE_POS_GAIN * e_off)
        delta = max(-self.MAX_STEER, min(self.MAX_STEER, delta))
        # The feedback trajectory is NOT the checked two-arc path, so enforce
        # the SAME envelope it was validated against (body corners on the
        # pavement and off the oncoming half) with a one-step look-ahead:
        # if integrating this delta for dt would push any corner outside,
        # ease off the wheel until it won't - exactly what a driver does
        # when the nose swings too close to oncoming traffic.
        if delta != 0.0 and car.speed < -1e-3:
            kerb_lim = seg.width / 2.0 - self.PARK_REVERSE_KERB_MARGIN_M
            centre_lim = -self.PARK_CENTRELINE_MARGIN_M
            corners = self._park_corner_offsets()

            def inside(d: float) -> bool:
                rate = (car.speed / self.WHEELBASE) * math.tan(d)
                psi_n = psi_car + rate * dt
                v_n = v_car + math.sin(psi_car) * car.speed * dt
                for (lf, lr) in corners:
                    lat = v_n + lf * math.sin(psi_n) + lr * math.cos(psi_n)
                    if lat > kerb_lim or lat < centre_lim:
                        return False
                return True

            for f in (1.0, 0.75, 0.5, 0.35, 0.2, 0.0):
                if inside(delta * f):
                    delta *= f
                    break
        car.steer_angle = delta

        # --- bicycle kinematics (signed speed) ---
        # The rotation rate is proportional to speed by construction
        # (rate = v/L * tan(delta)): a slowing car rotates slower, and at a
        # standstill it does not rotate at all - there is no in-place turn
        # to guard against. Freezing the rate below 0.3 m/s instead killed
        # the final straightening exactly when the roll-out began (measured:
        # heading locked 6.5 deg nose-in for the last 0.3 m of the line).
        max_rate = self.A_LAT_MAX / max(abs(car.speed), 0.1)
        desired_rate = (car.speed / self.WHEELBASE) * math.tan(delta)
        rate = max(-max_rate, min(max_rate, desired_rate))
        car.heading = (car.heading + math.degrees(rate) * dt) % 360
        rad = math.radians(car.heading)
        car.x += math.sin(rad) * car.speed * dt * PPPM
        car.y += math.cos(rad) * car.speed * dt * PPPM
        self._sync_segment()

        self.park_phase = 'reverse'
        if PARK_DEBUG:
            # Also watch the swept body corners: the feedback trajectory is
            # NOT the checked path, so verify it stays inside the same
            # envelope (kerb margin + oncoming-lane margin).
            kerb_lim = seg.width / 2.0 - self.PARK_REVERSE_KERB_MARGIN_M
            centre_lim = -self.PARK_CENTRELINE_MARGIN_M
            worst_k = min(v_car + lf * math.sin(psi_car)
                          + lr * math.cos(psi_car)
                          for (lf, lr) in self._park_corner_offsets())
            worst_o = max(v_car + lf * math.sin(psi_car)
                          + lr * math.cos(psi_car)
                          for (lf, lr) in self._park_corner_offsets())
            if worst_k < centre_lim or worst_o > kerb_lim:
                print(f"[RVWARN] n={getattr(self, '_rv_dbg', 0)} "
                      f"corner out of envelope: [{worst_k:.2f}, {worst_o:.2f}] "
                      f"lims [{centre_lim:.2f}, {kerb_lim:.2f}]")
            self._rv_dbg = getattr(self, '_rv_dbg', 0) + 1
            print(f"[RVDBG] n={self._rv_dbg} s={self._s:.2f} "
                  f"d_rem={d_rem:.2f} v={car.speed:+.3f} "
                  f"psi={(math.degrees(psi_car)):+.2f} e={e_off:+.3f} "
                  f"delta={math.degrees(delta):+.1f}")
        if (converged or d_rem <= 0.05) \
                and abs(car.speed) < self.PARK_STANDSTILL_M_S:
            car.speed = 0.0
            self._reverse_park = None
            self._parked = True
            self.park_phase = 'stopped'
            print("✅ Reverse-in parking complete - standing at the kerb\n")

    # ---- U-turn (Wenden) - docs/DRIVING_MANEUVERS.md §5 ----

    @property
    def uturn_active(self) -> bool:
        return self._uturn_active

    def _generate_uturn(self):
        """Generate the U-turn reference line from the car's current state.

        Returns (pts_px, stops_s, reverse_zone, kappa_pts, hdg_pts) or None
        when the maneuver
        cannot be executed safely here. NEVER returns a line whose body
        corners would leave the pavement - an infeasible U-turn is aborted,
        not driven (the hard rules apply to the car, so they apply to the
        planned line with a small margin on top).

        Local frame: t_hat = direction of travel along the segment,
        r_hat = its right side. The road is treated as a straight strip of
        width seg.width centred on the segment (true for every case this
        maneuver is used on; junctions are excluded by the room checks).
        """
        net = self.network
        car = self.car
        seg = net.segments[car.seg_idx]
        if car.forward:
            tx, ty = seg.x2 - seg.x1, seg.y2 - seg.y1
        else:
            tx, ty = seg.x1 - seg.x2, seg.y1 - seg.y2
        tl = math.hypot(tx, ty)
        if tl < 1e-6:
            return None
        tx, ty = tx / tl, ty / tl
        rx, ry = ty, -tx                      # right vector (t rotated 90 deg cw)
        h_travel = math.atan2(tx, ty)          # heading along the travel dir
        # The car must be roughly aligned with the road to turn around on it.
        hdg_err = abs((car.heading - math.degrees(h_travel) + 180.0) % 360.0 - 180.0)
        if hdg_err > 45.0:
            return None
        width = seg.width
        c = config.kerb_offset_m(width)        # kerb position of the car CENTRE
        lane0 = min(self.LANE_OFFSET_M, c)
        # Local origin: the point on the CENTERLINE closest to the car, so
        # that pt(lat, along)'s lat is measured from the centerline (the car
        # itself sits at lat = lat0, which is NOT zero).
        p0x, p0y = car.x / PPPM, car.y / PPPM   # work in metres, convert at end
        lat0 = ((car.x - seg.x1) * rx + (car.y - seg.y1) * ry) / PPPM
        p0x -= rx * lat0
        p0y -= ry * lat0

        def pt(lat: float, along: float):
            return (p0x + tx * along + rx * lat, p0y + ty * along + ry * lat)

        # Body corners relative to the rear axle (metres) - same geometry
        # as the sprite / on-road box (config axle offsets).
        front_bumper = self.WHEELBASE + (config.CAR_LENGTH / 2.0
                                         - config.FRONT_AXLE_OFFSET_M)
        rear_bumper = config.CAR_LENGTH / 2.0 - config.REAR_AXLE_OFFSET_M
        half_wid = config.CAR_WIDTH / 2.0

        def corners_ok(x: float, y: float, h: float) -> bool:
            f = (math.sin(h), math.cos(h))
            r = (math.cos(h), -math.sin(h))
            for lf in (front_bumper, -rear_bumper):
                for lr in (half_wid, -half_wid):
                    cx_ = x + f[0] * lf + r[0] * lr
                    cy_ = y + f[1] * lf + r[1] * lr
                    # Lateral distance from the segment CENTERLINE (not from
                    # P0, which itself sits off-center on the kerb side -
                    # measuring from P0 would be wrong on both sides).
                    lat = ((cx_ * PPPM - seg.x1) * rx
                           + (cy_ * PPPM - seg.y1) * ry) / PPPM
                    if abs(lat) > width / 2.0 - self.UTURN_LINE_MARGIN_M:
                        return False
            return True

        single = width >= self.UTURN_SINGLE_SWING_MIN_WIDTH_M
        k = math.tan(self.MAX_STEER) / self.WHEELBASE   # rad per metre
        arc_ds = 0.025

        def worst_corner_lat(line_pts):
            """Max |lateral| reach of any body corner on the line (m)."""
            worst = 0.0
            for (px_, py_, ph) in line_pts:
                f = (math.sin(ph), math.cos(ph))
                r = (math.cos(ph), -math.sin(ph))
                for lf in (front_bumper, -rear_bumper):
                    for lr in (half_wid, -half_wid):
                        cx_ = px_ + f[0] * lf + r[0] * lr
                        cy_ = py_ + f[1] * lf + r[1] * lr
                        lat = ((cx_ * PPPM - seg.x1) * rx
                               + (cy_ * PPPM - seg.y1) * ry) / PPPM
                        worst = max(worst, abs(lat))
            return worst

        def build_line(s0):
            """Step-1 lateral blend to s0, the swing arc(s), and the tail.
            Returns (pts, stop_idx, reverse_zone, n1, n_t).

            The blends carry their TRUE tangent heading: a car on a slanted
            path points along the tangent, and its front corner swings out
            further than any nominal-heading check would see - planning with
            nominal headings silently produces lines that clip the kerb."""
            l1 = self.UTURN_STEP1_LEN_M
            dlat = s0 - lat0
            blend = abs(dlat) >= 0.3
            n1 = int(l1 / 0.25)
            line: list[tuple[float, float, float]] = []
            stop_idx_: list[int] = []
            for i in range(n1 + 1):
                d = i * 0.25
                t = min(1.0, d / l1) if blend else 0.0
                sm = t * t * (3.0 - 2.0 * t)             # smoothstep
                line.append((pt(lat0 + dlat * sm, d) + (h_travel,)))
            x, y = pt(s0, l1)
            h = h_travel
            if not single:
                stop_idx_.append(len(line) - 1)          # spec 5b step 1: stop at kerb

            def arc_forward(x_, y_, h_, dh_target):
                travelled = 0.0
                while travelled < dh_target / k - 1e-9:
                    h_ -= k * arc_ds                     # left turn: heading decreases
                    x_ += math.sin(h_) * arc_ds
                    y_ += math.cos(h_) * arc_ds
                    travelled += arc_ds
                    line.append((x_, y_, h_))
                return x_, y_, h_

            def arc_reverse(x_, y_, h_, dh_total_target, dh_total_now):
                while dh_total_now < dh_total_target - 1e-9:
                    dh = k * arc_ds
                    # reverse + right steer keeps rotating LEFT (signed-v
                    # bicycle kinematics: dh/ds_path = sign(v) * tan(d)/L)
                    h_ -= dh
                    x_ -= math.sin(h_) * arc_ds          # moving BACKWARD
                    y_ -= math.cos(h_) * arc_ds
                    dh_total_now += dh
                    line.append((x_, y_, h_))
                return x_, y_, h_

            reverse_zone_ = None
            if single:
                # §5a: one continuous full-left arc across the road to 180 deg.
                x, y, h = arc_forward(x, y, h, math.pi)
            else:
                # §5b three-point: F(60) stop, R(to 90 total) stop, F(to 180).
                th2 = math.radians(self.UTURN_TH2_DEG)
                th3 = math.radians(self.UTURN_TH3_DEG)
                x, y, h = arc_forward(x, y, h, th2)
                s_a2 = len(line) - 1
                stop_idx_.append(s_a2)
                x, y, h = arc_reverse(x, y, h, th3, th2)
                s_a3 = len(line) - 1
                stop_idx_.append(s_a3)
                x, y, h = arc_forward(x, y, h, math.pi - th3)
                reverse_zone_ = (s_a2, s_a3)

            # --- tail: straight out in the NEW direction, blend to lane offset ---
            fx, fy = math.sin(h), math.cos(h)            # new travel direction
            rnx, rny = fy, -fx                           # new right vector
            lat_now = (x - p0x) * rnx + (y - p0y) * rny
            n_t = int(self.UTURN_TAIL_LEN_M / 0.25)
            for i in range(1, n_t + 1):
                d = i * 0.25
                t = min(1.0, d / 6.0)
                sm = t * t * (3.0 - 2.0 * t)
                lat = lat_now + (lane0 - lat_now) * sm
                # A4 already sits at lateral lat_now; the offset from A4 is
                # (lat - lat_now), not lat.
                line.append((x + fx * d + rnx * (lat - lat_now),
                             y + fy * d + rny * (lat - lat_now), h))

            # True-tangent headings on the blends (see above).
            for i_ in range(len(line) - 1):
                if i_ <= n1 or i_ >= len(line) - n_t:
                    dx_ = line[i_ + 1][0] - line[i_][0]
                    dy_ = line[i_ + 1][1] - line[i_][1]
                    line[i_] = (line[i_][0], line[i_][1], math.atan2(dx_, dy_))
            return line, stop_idx_, reverse_zone_, n1, n_t

        if single:
            # §5a: the 180 deg swing shifts the car exactly 2R laterally, so
            # starting the arc too far out clips the approach blend while
            # starting it too far in clips the tail blend. Search for the
            # arc-start lateral position that maximizes the minimum corner
            # clearance - an honest plan keeps margin, a line whose corners
            # hug the kerb WILL be clipped by the follower.
            limit_lat = (width / 2.0 - self.UTURN_LINE_MARGIN_M
                         - self.UTURN_BLEND_TRACKING_MARGIN_M)
            best = None                                  # (worst, pts, n1, n_t)
            s0_lo, s0_hi = sorted((lat0, c))
            n_cand = 15
            for j in range(n_cand + 1):
                s0 = s0_lo + (s0_hi - s0_lo) * (j / n_cand)
                cand, _si, _rz, cn1, cnt = build_line(s0)
                w_ = worst_corner_lat(cand)
                if w_ <= limit_lat and (best is None or w_ < best[0]):
                    best = (w_, cand, cn1, cnt)
            if best is None:
                return None
            pts, stop_idx, reverse_zone = best[1], [], None
            n1, n_t = best[2], best[3]
        else:
            # §5b: the swing must start at the kerb (spec 5b).
            pts, stop_idx, reverse_zone, n1, n_t = build_line(c)

        # --- longitudinal room: the arcs extend ahead of P0 (original travel
        # direction), the tail extends behind it. Both must fit the segment.
        if car.forward:
            p_behind = net.nodes[seg.start_node]
            p_ahead = net.nodes[seg.end_node]
        else:
            p_behind = net.nodes[seg.end_node]
            p_ahead = net.nodes[seg.start_node]
        room_ahead = ((p_ahead[0] - car.x) * tx + (p_ahead[1] - car.y) * ty) / PPPM
        room_behind = ((car.x - p_behind[0]) * tx + (car.y - p_behind[1]) * ty) / PPPM
        alongs = [((px_ - p0x) * tx + (py_ - p0y) * ty) for (px_, py_, _) in pts]
        if max(alongs) + 1.0 > room_ahead or -min(alongs) + self.CAR_LENGTH_M > room_behind:
            return None

        # --- hard-rule check: every body corner of the whole line must stay
        # on the pavement (with a small planning margin on top).
        for (px_, py_, ph) in pts:
            if not corners_ok(px_, py_, ph):
                return None

        # Per-point curvature (rad/m along the traversal). The polyline's
        # spatial direction flips 180 deg at every cusp, so RefLine's windowed
        # curvature reads garbage there - but we generated this line and know
        # its exact shape: -k on every arc (heading decreases), 0 elsewhere.
        kappa_pts = [0.0] * len(pts)
        if single:
            ranges = ((n1 + 1, len(pts) - n_t - 1),)
        else:
            ranges = ((n1 + 1, s_a2), (s_a2 + 1, s_a3),
                      (s_a3 + 1, len(pts) - n_t - 1))
        for lo, hi in ranges:
            for i in range(lo, hi + 1):
                kappa_pts[i] = -k

        # Required NOSE heading at every point: the polyline's true tangent,
        # plus 180 deg on the reverse branch (there the nose points AWAY from
        # the direction of travel). Recomputed from the geometry so that the
        # lateral blends (step 1, tail) carry their real tangent - the
        # nominal h_travel stored in the tuples would be wrong there and the
        # car would drive straight while the line drifts to the kerb.
        hdg_pts = [0.0] * len(pts)
        for i_ in range(len(pts) - 1):
            dx_ = pts[i_ + 1][0] - pts[i_][0]
            dy_ = pts[i_ + 1][1] - pts[i_][1]
            hdg_pts[i_] = math.atan2(dx_, dy_)
        hdg_pts[-1] = hdg_pts[-2]
        if not single:
            for i_ in range(s_a2 + 1, s_a3 + 1):
                hdg_pts[i_] += math.pi

        pts_px = [(px_ * PPPM, py_ * PPPM) for (px_, py_, _) in pts]
        ref = RefLine(pts_px)
        # Map the stop point indices to arc lengths (metres).
        stops_s = [ref.cum[i] for i in stop_idx]
        rev_zone_s = None
        if reverse_zone is not None:
            rev_zone_s = (ref.cum[reverse_zone[0]], ref.cum[reverse_zone[1]])
        return pts_px, stops_s, rev_zone_s, kappa_pts, hdg_pts

    def _build_uturn_profile(self, ref: RefLine, reverse_zone) -> list[float]:
        """SIGNED v_max per metre for the U-turn line.

        Negative on the reverse step. The cap is UTURN_SPEED_MAX on the
        maneuver itself (the spec's 5-10 km/h), ramping up along the tail by
        the car's own acceleration; the curvature cap and the forward
        braking-reachability pass are the same as for normal routes.
        """
        n = max(2, int(ref.total) + 1)
        s_tail_start = ref.total - self.UTURN_TAIL_LEN_M
        prof: list[float] = []
        for i in range(n):
            v = min(self.V_MAX, math.sqrt(
                self.UTURN_SPEED_MAX ** 2
                + 2 * self.A_CRUISE * max(0.0, i - s_tail_start)))
            k_ = abs(ref.curvature_at(min(ref.total, i + 0.5)))
            if k_ > 1e-4:
                v = min(v, math.sqrt(
                    self.A_LAT_MAX * self.A_LAT_PLAN_FRACTION / k_))
            prof.append(v)
        for i in range(n - 2, -1, -1):
            prof[i] = min(prof[i], math.sqrt(prof[i + 1] ** 2 + 2 * self.A_BRAKE))
        if reverse_zone is not None:
            # Index boundaries: negative EXACTLY between the stop points
            # (int() would start up to 1 m early and end up to 1 m late,
            # which flips the car's direction before/after the cusps).
            a = max(0, math.ceil(reverse_zone[0]))
            b = min(n, math.floor(reverse_zone[1]) + 1)
            for i in range(a, b):
                prof[i] = -abs(prof[i])
        return prof

    def _start_uturn(self) -> bool:
        if abs(self.car.speed) > self.UTURN_MAX_ENTRY_SPEED_M:
            print("\n⚠️  U-turn requested but the car is moving too fast "
                  f"({self.car.speed * 3.6:.0f} km/h) - brake first, then retry\n")
            return False
        gen = self._generate_uturn()
        if gen is None:
            print("\n⚠️  U-turn requested but not feasible here "
                  "(road width / alignment / room) - ignoring the request\n")
            return False
        pts_px, stops_s, rev_zone, kappa_pts, hdg_pts = gen
        self._ref = RefLine(pts_px)
        self._uturn_profile = self._build_uturn_profile(self._ref, rev_zone)
        self._uturn_stops = stops_s
        self._uturn_kappa = kappa_pts
        self._uturn_hdg = hdg_pts
        # Arc boundaries (where the stored curvature jumps): pursuit and
        # heading alignment must never aim PAST one of these, or the car
        # pre-turns into a minimum-radius arc it cannot correct on (full
        # lock is already required to stay on it). For a single swing this
        # is what keeps the entry clean - there are no stops to clamp to.
        jump_s = [self._ref.cum[i_] for i_ in range(1, len(kappa_pts))
                  if kappa_pts[i_] != kappa_pts[i_ - 1]]
        self._uturn_lookahead_clamps = sorted(set(stops_s) | set(jump_s))
        self._uturn_stop_ptr = 0
        self._uturn_state = 'drive'
        self._uturn_hold_t = 0.0
        self._uturn_approach_dir = 0
        self._uturn_mode = 'fwd'
        self._uturn_stall_t = 0.0
        self._uturn_release_force = 0.0
        self._s = 0.0
        self._uturn_active = True
        kind = "single swing (§5a)" if not stops_s else "three-point (§5b)"
        print(f"\n🔄 U-TURN (Wenden) started: {kind}, line {self._ref.total:.1f} m, "
              f"{len(stops_s)} full stop(s)\n")
        return True

    def _finish_uturn(self):
        """Hand back to normal driving: the car is now on the same road,
        facing the opposite way, in its lane - a plain route rebuild picks
        up from there (car.forward has flipped, so the route just mirrors).
        """
        print("\n✅ U-turn complete - resuming normal driving\n")
        self._uturn_active = False
        self._ref = None
        self._route = []
        self._route_key = None
        self._profile = []
        self._s = 0.0

    def _update_uturn(self, dt: float, control: dict):
        """Per-frame U-turn execution (replaces the normal update body)."""
        car = self.car
        ref = self._ref
        # Tight window: the U-turn line folds back on itself, and the
        # spatially nearest point can be on a different branch than the one
        # the car is driving (that mis-projection deadlocks the stops).
        self._s = project_s(ref, car.x, car.y, self._s,
                            window=0.3, global_fallback=False)
        prof = self._uturn_profile
        v_target = prof[int(max(0.0, min(len(prof) - 1, self._s)))]

        # --- full stops between the steps (spec: "Stoppen") ---
        # A stop is consumed ONLY by actually holding at it (below). Safety
        # net: if we have clearly overshot one (missed brake), skip it so we
        # don't keep targeting a point that is behind us.
        while (self._uturn_stop_ptr < len(self._uturn_stops)
               and self._s >= self._uturn_stops[self._uturn_stop_ptr] + 1.0):
            self._uturn_stop_ptr += 1
        ns = (self._uturn_stops[self._uturn_stop_ptr]
              if self._uturn_stop_ptr < len(self._uturn_stops) else None)
        # Brake by PHYSICAL distance to the stop point, not by s-distance:
        # on a folded line the projection can slide onto another branch and
        # s lies, but the car's real position does not.
        braking_to_stop = False
        if (ns is not None and self._uturn_state == 'drive'
                and self._uturn_release_force == 0.0):
            spx, spy = ref.point_at(ns)
            d_stop = math.hypot(car.x - spx, car.y - spy) / PPPM
            # 1 m margin: the car may sit up to ~0.5-0.8 m OFF the line
            # laterally, and d_stop is measured to a point - with A_BRAKE =
            # 10 m/s^2 the bare v^2/2a + 0.3 threshold (0.69 m at 2.8 m/s)
            # would never trigger against that offset.
            braking_to_stop = (d_stop <= car.speed ** 2 / (2.0 * self.A_BRAKE)
                               + 1.0)
        if self._uturn_state == 'holding':
            v_target = 0.0
            self._uturn_hold_t += dt
            if self._uturn_hold_t >= self.UTURN_HOLD_S:
                # Released. The car is ON the cusp (the creep phase drove it
                # there), so the line PAST this stop is where we go next.
                # (A1: forward; A2: reverse; A3: forward.) The profile may
                # lag the cusp by up to ~0.3 m (it switches at integer s),
                # so force the release direction until the LOCAL profile
                # agrees with it and sustains it on its own.
                i_past = min(len(prof) - 1, int(ns) + 2)
                self._uturn_release_force = (-0.5 if prof[i_past] < 0.0
                                             else +0.5)
                self._uturn_approach_dir = 0
                self._uturn_stop_ptr += 1
                self._uturn_state = 'drive'
        elif braking_to_stop:
            v_target = 0.0
            if self._uturn_approach_dir == 0:
                self._uturn_approach_dir = (1 if car.speed >= 0 else -1)
            if abs(car.speed) < 0.1:
                # Actually stopped now. If we are still SHORT of the stop
                # point, creep onto it first: each step of a three-point turn
                # must start EXACTLY at its cusp - the arcs are minimum
                # radius, so starting 1 m early traces a different circle
                # and misses the next stop entirely. (A real driver does
                # exactly this: brake, inch up to the edge, stop.)
                if ns is not None and self._s < ns - 0.05:
                    self._uturn_state = 'creeping'
                    self._uturn_hold_t = 0.0
                else:
                    self._uturn_state = 'holding'
                    self._uturn_hold_t = 0.0
        elif self._uturn_state == 'creeping':
            # Inch up to the cusp in the direction we were approaching it.
            v_target = 0.5 * self._uturn_approach_dir
            if ns is not None and self._s >= ns - 0.05:
                self._uturn_state = 'holding'
                self._uturn_hold_t = 0.0

        # Forced release direction right after a stop, until the local speed
        # profile agrees with it (see above).
        if self._uturn_release_force != 0.0:
            if self._uturn_release_force < 0.0:
                v_target = min(v_target, self._uturn_release_force)
            else:
                v_target = max(v_target, self._uturn_release_force)
            i_now = int(max(0.0, min(len(prof) - 1, self._s)))
            if ((self._uturn_release_force < 0.0 and prof[i_now] < 0.0)
                    or (self._uturn_release_force > 0.0
                        and prof[i_now] > 0.0)):
                self._uturn_release_force = 0.0

        # --- pursuit mode from the sign of the target speed (the zero zone
        # between stops keeps the last mode, so there is no flip-flop) ---
        if v_target < -0.05:
            self._uturn_mode = 'rev'
        elif v_target > 0.05:
            self._uturn_mode = 'fwd'

        # --- steering: curvature feed-forward on arcs, pure pursuit on
        # straights.
        # The arcs are MINIMUM radius: a car on the line needs EXACTLY
        # MAX_STEER there, so point-chasing has zero margin to correct with.
        # Worse, in reverse the pursuit target sits along the direction of
        # motion, so the law reads ~0-25 deg - BLENDING it in would
        # under-steer the arc and let the nose fall behind (measured: 13 deg
        # by A3 -> 1.4 m outside the line on step 4). delta = atan(L*kappa)
        # makes a kinematic bicycle trace an arc of radius 1/kappa exactly
        # (in reverse the sign mirrors); a constant cusp-entry heading offset
        # then only produces a parallel arc a few cm away. On straights
        # kappa=0 and plain pursuit trims lateral error.
        # (Stored per-point curvature: RefLine's windowed curvature breaks at
        # the cusps, where the polyline's spatial direction flips 180 deg.)
        i = bisect.bisect_left(ref.cum, self._s)
        kappa = self._uturn_kappa[max(0, min(len(self._uturn_kappa) - 1, i))]
        if abs(kappa) > 0.05:
            delta = math.atan(self.WHEELBASE * kappa)
            if self._uturn_mode == 'rev':
                delta = -delta
        else:
            # Never aim PAST the next cusp (stop point): a target on the
            # next branch (the arc after A1) makes the car start turning
            # early, entering the minimum-radius arc with a heading error it
            # can never correct (full lock is already required to stay on
            # it). Use the nearest stop ahead, NOT ns - after a release the
            # pointer has already advanced past the cusp that still lies in
            # front of the car.
            lookahead = 2.0 + 0.15 * abs(car.speed)
            for st_ in self._uturn_lookahead_clamps:
                if st_ > self._s:
                    lookahead = min(lookahead, max(0.1, st_ - self._s))
                    break
            tx, ty = ref.point_at(self._s + lookahead)
            dx = (tx - car.x) / PPPM
            dy = (ty - car.y) / PPPM
            h = math.radians(car.heading)
            if self._uturn_mode == 'fwd':
                local_f = dx * math.sin(h) + dy * math.cos(h)
                if local_f < 0.5:
                    local_f = 0.5
                local_r = dx * math.cos(h) - dy * math.sin(h)
                delta_pp = math.atan2(local_r, local_f)
            else:
                # Reversing: aim the REAR of the car at the target point. In
                # reverse a right steer swings the rear to the LEFT (bicycle
                # kinematics with signed v), so the pursuit law mirrors.
                local_f = -(dx * math.sin(h) + dy * math.cos(h))
                if local_f < 0.5:
                    local_f = 0.5
                local_r = -(dx * math.cos(h) - dy * math.sin(h))
                delta_pp = -math.atan2(local_r, local_f)
            # Blend in heading alignment to the line's own target nose
            # heading: at maneuver speeds plain pursuit limit-cycles on a
            # straight (measured ~5 deg wobble at the A1 entry), and the arcs
            # inherit whatever heading error is left. The sample point is
            # clamped BEFORE the next cusp - sampling past it makes the car
            # pre-turn into the arc while still on the straight (measured:
            # 7 deg of lead creeping up to A1).
            s_h = self._s + 0.3
            for st_ in self._uturn_lookahead_clamps:
                if st_ > self._s:
                    s_h = min(s_h, st_ - 0.05)
                    break
            i_h = bisect.bisect_left(ref.cum, max(0.0, s_h))
            h_target = math.degrees(self._uturn_hdg[
                max(0, min(len(self._uturn_hdg) - 1, i_h))]) % 360.0
            delta_align = (h_target - car.heading + 180.0) % 360.0 - 180.0
            delta = 0.5 * delta_pp + 0.5 * delta_align
        delta = max(-self.MAX_STEER, min(self.MAX_STEER, delta))

        # --- longitudinal: throttle/brake toward the SIGNED profile ---
        # The steering-angle accel gate is deliberately NOT applied here:
        # the profile already caps every arc at UTURN_SPEED_MAX, and with it
        # the car would stall on full-lock arcs (gate closed -> no throttle).
        if car.speed > v_target + 0.05:
            car.speed = max(v_target, car.speed - self.A_BRAKE * dt)
            car._braking = True
        elif car.speed < v_target - 0.05:
            if v_target > 0:
                car.speed = min(v_target, car.speed + self.A_CRUISE * dt)
            else:
                car.speed = max(v_target, car.speed - self.A_CRUISE * dt)
            car._braking = False
        else:
            car._braking = False
        car.speed = max(-5.0, min(self.V_MAX, car.speed))
        car.target_speed = car.speed

        # --- bicycle kinematics (signed speed: reverse falls out for free) ---
        max_rate = (self.A_LAT_MAX / abs(car.speed)) if abs(car.speed) > 0.3 else 0.0
        desired_rate = (car.speed / self.WHEELBASE) * math.tan(delta)
        rate = max(-max_rate, min(max_rate, desired_rate))
        car.heading = (car.heading + math.degrees(rate) * dt) % 360
        rad = math.radians(car.heading)
        car.x += math.sin(rad) * car.speed * dt * PPPM
        car.y += math.cos(rad) * car.speed * dt * PPPM

        self._sync_segment()

        # --- completion / stall guard ---
        if self._s >= ref.total - 1.0:
            self._finish_uturn()
            return
        if (self._uturn_state == 'drive' and abs(car.speed) < 0.05
                and abs(v_target) > 0.2):
            self._uturn_stall_t += dt
            if self._uturn_stall_t > self.UTURN_STALL_ABORT_S:
                print("⚠️  U-turn stalled (no progress for "
                      f"{self.UTURN_STALL_ABORT_S:.0f} s) - aborting the maneuver")
                # The car recognises it cannot continue: make that VISIBLE
                # (hazard lights, min 5 s display, logged) instead of just
                # silently falling back to normal driving.
                if hasattr(car.driver, 'set_hazard'):
                    car.driver.set_hazard(
                        True, reason="U-turn stalled - cannot continue")
                self._finish_uturn()
        else:
            self._uturn_stall_t = 0.0

    # ---- per-frame update ----

    def update(self, dt: float, control: dict):
        car = self.car
        if self._uturn_active:
            self._update_uturn(dt, control)
            return
        # Reverse-in parking (docs §1b) owns the car while it runs.
        if self._reverse_park is not None:
            self._update_reverse_park(dt)
            return
        # Parked: stay parked. The destination has been reached and the car
        # is standing at the kerb - it does not roll off again on its own.
        if self._parked:
            if control.get("brake") or car.speed != 0.0:
                car.speed = 0.0
            self.park_phase = 'stopped'
            self.lane_change_signal = None
            self._merge_episode = None
            car.target_speed = 0.0
            return
        # U-turn request (one-shot flag on the driver, set by keyboard/API).
        d = car.driver
        if d is not None and getattr(d, 'uturn_requested', False):
            d.uturn_requested = False
            self._start_uturn()
            if self._uturn_active:
                return
        # Pull-out mode: first ~2 seconds after spawn → max left steer + slow speed
        pulling_out = (self._pull_out_frames > 0)
        if pulling_out:
            self._pull_out_frames -= 1
        # Brake & park plan (spec §1): stateless per-tick evaluation of
        # (distance to stop point, current speed). When active it owns the
        # longitudinal target; pulling_over derives from its phase (the
        # reference line shifts to the kerb for the swerve zone). The stop
        # point is expressed for the car's REFERENCE POINT (the rear axle,
        # which is what _s tracks): at a red flag the car's FRONT BUMPER
        # rests on the flag (spec §1), so the axle stops the front overhang
        # short of it; at a dead end the whole car stops a car length short
        # of the pavement edge so the front corners stay on the road.
        self.park_phase = 'none'
        plan = None
        d_stop = None
        self._decide_park_style()
        if self._has_destination() and not pulling_out \
                and self._ref is not None:
            stop_s = self._ref.total - self._stop_margin()
            d_stop = stop_s - self._park_s()
            plan = self._parking_plan(d_stop, car.speed)
            if plan is not None:
                # Stopped at the destination: blinker off (spec §1).
                self.park_phase = ('stopped'
                                   if d_stop <= 0.5 and car.speed < 0.3
                                   else plan[0])
            # Standing at the staging point: back into the space (§1b).
            # The roll-out can come to rest up to ~0.3 m SHORT of the stop
            # point (the tail's constant decel engages below 0.3 m/s, i.e.
            # while d/tau still says "keep rolling"), so the threshold must
            # swallow that variance - measured live: a stop at d_stop=0.22
            # sat forever two centimetres outside a 0.2 gate and the car
            # never reversed. The reverse path is anchored to the car's ACTUAL
            # pose (it re-solves the tuck from here), so starting slightly
            # early just means a marginally longer back-up.
            if (self._park_style == 'reverse' and plan is not None
                    and d_stop <= 0.5 and abs(car.speed) < 0.05):
                if self._start_reverse_park():
                    self._update_reverse_park(dt)
                    return
                # Not reachable from here after all: stay where we are,
                # parked forwards (the pull-over has already been driven).
                self._park_style = 'forward'
                self._park_style_locked = True
                self._parked = True
                self.park_phase = 'stopped'
                car.speed = 0.0
                return
        pulling_over = plan is not None \
            and plan[0] in ('decel', 'swerve', 'final')
        # Lane-change blinker (user rule: ALWAYS signal before changing
        # lanes): on from MERGE_SIGNAL_AHEAD_M before the merge zone starts,
        # off once the car has settled onto the new line (past s1).
        self.lane_change_signal = None
        ep = self._merge_episode
        if ep is not None and self._ref is not None:
            s0, s1, direction = ep
            if s0 - self.MERGE_SIGNAL_AHEAD_M <= self._s < s1:
                self.lane_change_signal = direction
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
        d_to_stop = (d_stop if d_stop is not None
                     else ref.total - self._s)
        if pulling_over and d_to_stop < self.PARK_BLEND_START_M:
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
        if local_forward <= 0.0:
            # Aim point at or BEHIND us: happens in tight corners where the
            # s-projection (coarse window scan, ~0.5-1 m resolution) lags
            # the car while the high-curvature lookahead (0.5 m) is shorter
            # than that lag. Aiming at a point behind commands a hard
            # counter-steer and oscillates the car in place until it leaves
            # the road (measured: roundabout entry, test 17 - heading sawed
            # 190 -> 178 -> 210 deg at 9 km/h while s sat on a quantisation
            # step). Align with the line's tangent instead: that is what a
            # driver does when the aim point slips behind.
            lh = math.radians(ref.heading_at(self._s))
            delta = math.atan2(math.sin(lh - h), math.cos(lh - h))
        else:
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

        # Pull-over final straight (spec §1 "parallel ausrichten"): the
        # line is a constant offset here, so the job is no longer to chase
        # a point ahead but to END UP ON the line, parallel to it. Pure
        # pursuit cannot do that: it aims at a point, so it arrives at the
        # line still rotating and the car froze a few degrees nose-in
        # (measured 5-12 deg). Use a Stanley-type law instead - steer by
        # the heading error PLUS a cross-track term that vanishes as the
        # car reaches the line. Both terms go to zero together, which is
        # exactly "flush with the kerb and parallel to it".
        # This is a real turn: the car is still rolling and the
        # lateral-accel clamp below keeps the implied radius above the
        # physical minimum (no in-place rotation, per the project's rule).
        if pulling_over and d_to_stop < self.PARK_BLEND_END_M:
            ref_x, ref_y = ref.point_at(self._s)
            line_heading = ref.heading_at(self._s)
            lh = math.radians(line_heading)
            # Right-hand normal of the line, in screen coordinates
            # (x = sin(h), y = cos(h) is "forward").
            rnx, rny = math.cos(lh), -math.sin(lh)
            e_right = ((car.x - ref_x) * rnx + (car.y - ref_y) * rny) / PPPM
            heading_err = math.radians(
                (line_heading - car.heading + 180.0) % 360.0 - 180.0)
            cross = math.atan2(self.PARK_ALIGN_GAIN * e_right,
                               max(car.speed, 1.0))
            # Fade the cross term out over the last metre (see constant):
            # only the heading correction survives into the final roll-out.
            if d_to_stop < self.PARK_ALIGN_CROSS_FADE_START_M:
                cross *= max(0.0, min(1.0,
                            (d_to_stop - self.PARK_ALIGN_CROSS_FADE_END_M)
                            / (self.PARK_ALIGN_CROSS_FADE_START_M
                               - self.PARK_ALIGN_CROSS_FADE_END_M)))
            # Heading term with PARK_ALIGN_HDG_GAIN (see constant): the
            # 1:1 match is too slow for this window at creep speed.
            delta = max(-self.MAX_STEER, min(self.MAX_STEER,
                                             self.PARK_ALIGN_HDG_GAIN
                                             * heading_err - cross))

        # --- longitudinal: throttle/brake toward the speed profile ---
        # The profile ALREADY encodes the cruising speed on straights and
        # the (much lower) corner speed at bends, with a braking ramp into
        # each corner. The accelerator means "go as fast as the profile
        # allows" - it must NOT override the corner limit (that is exactly
        # what made the car barrel into corners at cruise and swing wide).
        accel = control.get("accelerate", False)
        brake = control.get("brake", False)
        brake_rate = self.A_BRAKE
        if plan is not None:
            # Inside the plan the deceleration never exceeds A_PARK (a real
            # driver modulates the pedal); full A_BRAKE only as an
            # emergency when even A_PARK can no longer stop in time.
            phase, v_plan = plan
            # The emergency only makes sense while the car still carries
            # real speed. At creep speed a few centimetres of overshoot
            # past the stop point used to flip this to A_BRAKE and slam
            # the car to a standstill from 2 km/h - the exact jerk spec §1
            # forbids, and nothing was gained by it.
            brake_rate = self.A_BRAKE if (
                d_stop is not None
                and car.speed > self.PARK_SWERVE_SPEED_M
                and d_stop < car.speed ** 2 / (2.0 * self.A_PARK) - 0.5
            ) else self.A_PARK
            if phase == 'lead':
                # Lead phase: HOLD the approach speed (table phase 1) -
                # unless the normal profile (a corner ahead) demands less.
                v_target = min(car.speed, self._target_speed(self._s))
            else:
                v_target = v_plan
            # Braking for a CORNER is not part of the parking manoeuvre and
            # must not be slowed down by its comfort limit. The plan can
            # engage 100 m before the destination - across a junction the
            # car still has to turn at - and capping the pedal at A_PARK
            # there meant it could no longer make the junction entry speed:
            # it arrived at a T-junction at 59 km/h instead of 29 and cut
            # the corner off the road (measured, tests 3/4).
            v_prof = self._target_speed(self._s)
            if v_prof <= v_target + 1e-3:
                v_target = min(v_target, v_prof)
                brake_rate = self.A_BRAKE
            if phase == 'final' and car.speed < self.PARK_ROLL_END_M_S:
                # Tail of the roll-out: v = d/tau decays exponentially and
                # would creep for another second at walking-pace/10. Close
                # it out with a small CONSTANT deceleration - still a
                # sixth of A_PARK, so the pedal is easing off, not
                # stamping - which reaches zero in ~0.5 s and stays there.
                v_target = 0.0
                brake_rate = min(brake_rate, self.PARK_ROLL_END_A)
            if phase == 'final':
                # Hands off the throttle for the roll-out. The target
                # shrinks with the remaining distance, and the projection
                # of that distance jitters by a centimetre or two, so an
                # active throttle chased it: the last second of the stop
                # sawed between 0.2 and 0.4 km/h at +-3 m/s^2. A driver
                # coming to a stop is not on the gas.
                accel = False
        else:
            # No active plan. Pedal held (suite protocol / human on W):
            # cruise at the profile speed - including keeping hard after
            # a target while off line, which is the recovery behaviour we
            # want DURING a scenario. Pedal released (after reset_controls()
            # once the test is finished): no throttle - ease down to a stop
            # at the comfort rate (engine braking + rolling resistance),
            # like a real car left in gear with the foot off the gas, so a
            # leftover test car comes to rest instead of rolling forever.
            if accel:
                v_target = self._target_speed(self._s)
            else:
                v_target = 0.0
                brake_rate = min(brake_rate, self.A_PARK)
        if brake:
            # External brake (keyboard S / API safety net) always wins.
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
        # ...but never while parking: the plan's roll-out target keeps
        # falling, and the creep boost pushed against it every other frame
        # (measured: the speed sawing between 0.2 and 0.4 km/h with +-3
        # m/s^2 for the last second - a visible shudder at the kerb).
        if accel and car.speed < self.CREEP_SPEED and plan is None:
            accel_scale = max(accel_scale, self.CREEP_SCALE)
        # Ease speed toward the target (accel / brake rates).
        if accel and car.speed < v_target:
            car.speed = min(v_target, car.speed + self.A_CRUISE * accel_scale * dt)
        elif car.speed > v_target:
            car.speed = max(v_target, car.speed - brake_rate * dt)
        car.speed = max(0.0, min(self.V_MAX, car.speed))
        # End of the parking roll-out: the distance-proportional target
        # decays exponentially and never reaches zero, so drop the last
        # 0.2 km/h once the car is at (or past) the stop point.
        if plan is not None and plan[0] == 'final' and d_stop is not None \
                and d_stop <= 0.3 and car.speed < self.PARK_STANDSTILL_M_S:
            car.speed = 0.0
        car.target_speed = car.speed
        car._braking = brake or (car.speed > v_target + 0.05)

        # End-of-stop wheel straightening (parking only): below the yaw-
        # clamp threshold (0.3 m/s, see the kinematics below) the car
        # physically cannot rotate, so a large held steering command only
        # reads as "fighting the wheel" while it rolls straight to its stop
        # (measured: -10 deg held over the last 0.5 s). Fade the command to
        # zero across [0.3, 0.5] m/s while the parking plan is active -
        # continuous, and free above 0.5 m/s where the rotation budget
        # actually lives. Other maneuvers (U-turn, reverse-in) keep their
        # full commands.
        if plan is not None and car.speed < 0.5:
            delta *= max(0.0, min(1.0, (car.speed - 0.3) / 0.2))

        # Expose the steering angle for the driver's mechanical blinker
        # auto-off (steered in + steered back = indicator cancels itself).
        car.steer_angle = delta

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
        self._pull_out_frames = 0   # spawn is in the driving position (see __init__)
        self._uturn_active = False
        self._uturn_profile = []
        self._uturn_stops = []
        self._uturn_stop_ptr = 0
        self._uturn_state = 'drive'
        self._uturn_hold_t = 0.0
        self._uturn_approach_dir = 0
        self._uturn_mode = 'fwd'
        self._uturn_stall_t = 0.0
        self._uturn_kappa = []
        self._uturn_hdg = []
        self._uturn_lookahead_clamps = []
        self._uturn_release_force = 0.0
        self._park_style = 'forward'
        self._park_style_locked = False
        self._park_tuck = 0.0
        self._park_stage_m = 0.0
        self._park_stage_short = False
        self._reverse_park = None
        self._parked = False
