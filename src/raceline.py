# Racing line - the fastest legal line through a route.
#
# The driving line is not a fixed lane offset. It is the solution to a
# constrained optimisation, stated exactly as the game's driving rules:
#
#   1. never leave the pavement          -> upper corridor bound
#   2. never enter the oncoming lane     -> lower corridor bound
#   3. be as fast as possible            -> the objective
#   4. use a racing line                 -> what (3) produces, not a feature
#
# Since v = sqrt(a_lat / kappa), "as fast as possible" means "as straight
# as possible": minimise the curvature of the line, subject to staying
# inside the corridor that rules 1 and 2 define. Rules 1 and 2 are
# therefore HARD BOUNDS - the optimiser cannot trade them away for speed,
# so a legal line is guaranteed by construction rather than detected
# afterwards by a validator.
#
# Why an optimiser and not a formula: the intuitive "swing out before the
# corner, cut the apex" line CANNOT be had by nudging the centreline
# sideways with a bump function. Offsetting a path laterally by o(s)
# changes its curvature by roughly -o''(s), so a local bump buys radius at
# the apex and pays for it with sharper curvature on both shoulders - the
# net effect is a TIGHTER line (measured: peak curvature 2-4x worse). The
# gain only appears when the whole approach and exit are free to reshape
# together, which is what the optimiser does and a formula cannot.
#
# The classic outside-apex-outside line is not encoded anywhere here. It
# falls out of the geometry: on a right-hand bend the solver drives the
# offset to the centreline side on entry, to the kerb at the apex, and
# back out on exit - and mirrors itself for a left-hander.

from __future__ import annotations

import math

from . import config

PPPM = config.PIXELS_PER_METER


# ----------------------------------------------------------------------
# banded linear algebra
# ----------------------------------------------------------------------
# Minimising sum((kappa0 - o'')^2) gives normal equations whose matrix is
# the square of a second-difference operator: symmetric, positive
# definite (with regularisation) and PENTADIAGONAL. Exploiting that band
# makes each solve O(n) instead of O(n^3), which is what keeps a whole
# route affordable - a dense solve on a 250-station route needs ~5M
# operations per active-set iteration and is far too slow in Python.

BW = 2  # half-bandwidth of the biharmonic normal equations

SAMPLE_M = 0.5          # station spacing of the driving line
CORRIDOR_PROBE_M = 2.0  # spacing at which the pavement is actually probed
JUNCTION_EXTRA_ROOM_M = 1.5  # extra outside room inside an intersection


def _band_solve(A: list[list[float]], b: list[float]) -> list[float]:
    """Solve A x = b for a symmetric positive-definite banded matrix.

    A is stored band-wise: A[i][k] is the matrix element (i, i - BW + k),
    with out-of-range entries ignored. No pivoting is needed (the system
    is SPD thanks to the regularisation term added by the caller).
    """
    n = len(b)
    a = [row[:] for row in A]
    x = b[:]

    # Forward elimination, touching only the band.
    for i in range(n):
        piv = a[i][BW]
        if abs(piv) < 1e-14:
            piv = a[i][BW] = 1e-14
        for r in range(i + 1, min(n, i + BW + 1)):
            k = BW - (r - i)                 # column i as seen from row r
            f = a[r][k] / piv
            if f == 0.0:
                continue
            for c in range(0, BW + 1):       # upper half of row i
                rc = k + c
                if rc <= 2 * BW:
                    a[r][rc] -= f * a[i][BW + c]
            x[r] -= f * x[i]

    # Back substitution.
    for i in range(n - 1, -1, -1):
        s = x[i]
        for c in range(1, BW + 1):
            j = i + c
            if j < n:
                s -= a[i][BW + c] * x[j]
        x[i] = s / a[i][BW]
    return x


def _base_at(base, i: int) -> float | None:
    """`base` may be a scalar or a per-station profile (list)."""
    if base is None:
        return None
    return base[i] if isinstance(base, (list, tuple)) else base


def min_curvature_offsets(kappa0: list[float], lo: list[float], hi: list[float],
                          ds: float, max_rounds: int = 40,
                          base: float | list | None = None) -> list[float]:
    """Lateral offsets minimising line curvature inside [lo, hi].

    Minimise  sum_i ( kappa0_i - (o_{i-1} - 2 o_i + o_{i+1}) / ds^2 )^2
    subject to lo_i <= o_i <= hi_i.

    Box constraints are handled by an active set: solve, clamp whatever
    escaped its bound with a stiff penalty (which preserves the band, so
    every round stays O(n)), and repeat until the solution is feasible.

    `base` (the nominal lane offset) breaks the tie on STRAIGHT sections,
    where the objective is flat and any constant offset is optimal: the
    line settles at `base` instead of drifting to the corridor centre. On
    a wide one-way (no centreline bound) the corridor centre IS the middle
    of the road - which would make the car abandon its lane position.
    `base` may also be a per-station PROFILE (list, same length as the
    stations): each station then settles at its own nominal offset - used
    for the merge-right blend before parking (docs §1 variant).
    """
    n = len(kappa0)
    if n < 5:
        return [max(lo[i], min(hi[i],
                               _base_at(base, i) if _base_at(base, i) is not None
                               else 0.5 * (lo[i] + hi[i]))) for i in range(n)]

    inv = 1.0 / (ds * ds)
    stencil = ((-1, -inv), (0, 2.0 * inv), (1, -inv))
    LAMBDA = 1e-6       # keeps the (otherwise singular) system definite
    PENALTY = 1e6       # stiffness used to pin an active constraint

    pinned: dict[int, float] = {}
    o = [0.0] * n
    for _ in range(max_rounds):
        A = [[0.0] * (2 * BW + 1) for _ in range(n)]
        b = [0.0] * n
        # Regularise toward the nominal lane position at EVERY station.
        # On curves the curvature term dominates by many orders of
        # magnitude, so this only matters where the objective is flat:
        # it pins the line at `base` (spec §3's normal position) instead
        # of leaving it to the solver's whim. Pulling ONLY the endpoints
        # is not enough: the stencil chain then solves a long straight to
        # a LINEAR profile between them - dipping through the middle of
        # the road (measured: 1.67 m at both ends, -0.11 m mid-line on a
        # 300 m one-way). base=None reproduces the old pull-toward-zero.
        for i in range(n):
            A[i][BW] += LAMBDA
            _b = _base_at(base, i)
            b[i] += LAMBDA * max(lo[i], min(hi[i],
                                            _b if _b is not None else 0.0))
        # One residual row per interior station.
        for i in range(1, n - 1):
            for da, ca in stencil:
                ia = i + da
                b[ia] += ca * kappa0[i]
                for db, cb in stencil:
                    ib = i + db
                    k = BW + (ib - ia)
                    if 0 <= k <= 2 * BW:
                        A[ia][k] += ca * cb
        # Pin the active constraints.
        for i, v in pinned.items():
            A[i][BW] += PENALTY
            b[i] += PENALTY * v
        # Endpoints are free to sit anywhere legal but must not float
        # away from the nominal lane position, which would tilt the whole
        # line (on straights this is what pins the line at `base`).
        for i in (0, n - 1):
            if i not in pinned:
                A[i][BW] += 1e-3
                _b = _base_at(base, i)
                tgt = (max(lo[i], min(hi[i], _b)) if _b is not None
                       else 0.5 * (lo[i] + hi[i]))
                b[i] += 1e-3 * tgt

        o = _band_solve(A, b)

        worst_i, worst_gap, worst_v = -1, 1e-9, 0.0
        for i in range(n):
            if i in pinned:
                continue
            if o[i] < lo[i] - 1e-9 and (lo[i] - o[i]) > worst_gap:
                worst_i, worst_gap, worst_v = i, lo[i] - o[i], lo[i]
            elif o[i] > hi[i] + 1e-9 and (o[i] - hi[i]) > worst_gap:
                worst_i, worst_gap, worst_v = i, o[i] - hi[i], hi[i]
        if worst_i < 0:
            break
        pinned[worst_i] = worst_v

    return [max(lo[i], min(hi[i], o[i])) for i in range(n)]


# ----------------------------------------------------------------------
# the legal corridor (rules 1 and 2)
# ----------------------------------------------------------------------

def _resample(pts, ds):
    """Resample a polyline (world px) at ~ds metres; return points + arc len."""
    cum = [0.0]
    for i in range(len(pts) - 1):
        cum.append(cum[-1] + math.hypot(pts[i + 1][0] - pts[i][0],
                                        pts[i + 1][1] - pts[i][1]) / PPPM)
    total = cum[-1]
    if total < 1e-6:
        return [pts[0]], [0.0], 0.0
    n = max(2, int(total / ds) + 1)
    out, S, j = [], [], 0
    for i in range(n):
        s = min(total, i * total / (n - 1))
        while j < len(cum) - 2 and cum[j + 1] < s:
            j += 1
        span = cum[j + 1] - cum[j]
        t = (s - cum[j]) / span if span > 1e-9 else 0.0
        out.append((pts[j][0] + t * (pts[j + 1][0] - pts[j][0]),
                    pts[j][1] + t * (pts[j + 1][1] - pts[j][1])))
        S.append(s)
    return out, S, total


def _normals_and_curvature(P, ds):
    n = len(P)
    N, K = [], []
    for i in range(n):
        a, b = P[max(0, i - 1)], P[min(n - 1, i + 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1.0
        N.append((dy / L, -dx / L))          # right normal
        if 0 < i < n - 1:
            h1 = math.atan2(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1])
            h2 = math.atan2(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1])
            K.append(((h2 - h1 + math.pi) % (2 * math.pi) - math.pi) / ds)
        else:
            K.append(0.0)
    K[0] = K[1] if n > 1 else 0.0
    K[-1] = K[-2] if n > 1 else 0.0
    return N, K


def _junction_node_per_station(network, P, radius_m=14.0):
    """Nearest junction centre (degree >= 3 node) for each station, or None.

    That node is the white dot the renderer paints at every real
    intersection. Going STRAIGHT or turning RIGHT it must stay on our left,
    which is just keep-right restated at the one place the centreline stops
    existing.

    Turning LEFT it must not. StVO 9(4): "Linksabbieger muessen einander
    voreinander abbiegen, sofern nicht die Verkehrslage oder die Gestaltung
    der Kreuzung ein Umeinanderfahren erfordert." The German default is
    VOREINANDER - opposing left-turners pass in front of one another,
    driver's side to driver's side, each turning before it reaches the
    centre. That puts the dot on their RIGHT. Umeinander (around the
    centre, dot on the left) is the exception, not the rule.

    Which is why a left turn could not satisfy a dot-on-the-left
    constraint without contorting to a 1.5 m radius: the constraint was
    simply wrong for that manoeuvre.

    Returns the node only where the route does NOT turn left there, so the
    caller constrains exactly the cases the rule covers.
    """
    r2 = (radius_m * PPPM) ** 2
    nodes = [xy for nid, xy in network.nodes.items()
             if network.node_degree.get(nid, 0) >= 3]
    def turns_left_at(k):
        """Signed heading change of the route across this node (+ = right)."""
        a, b = max(0, k - 20), min(len(P) - 1, k + 20)
        if b - a < 4:
            return False
        h1 = math.atan2(P[a + 1][0] - P[a][0], P[a + 1][1] - P[a][1])
        h2 = math.atan2(P[b][0] - P[b - 1][0], P[b][1] - P[b - 1][1])
        return ((h2 - h1 + math.pi) % (2 * math.pi) - math.pi) < math.radians(-30)

    out = []
    for k, (px, py) in enumerate(P):
        best, bd = None, r2
        for q in nodes:
            d2 = (px - q[0]) ** 2 + (py - q[1]) ** 2
            if d2 <= bd:
                bd, best = d2, q
        out.append(None if (best is not None and turns_left_at(k)) else best)
    return out


def legal_corridor(network, P, N, station_props, junction_nodes=None):
    """Per-station [lo, hi] lateral offsets that satisfy rules 1 and 2.

    hi (rule 1): the furthest RIGHT the car can sit with its whole body
        still on pavement. Taken from the paved polygon eroded by half the
        car's width plus the shared edge tolerance, so it automatically
        respects corner rounding - the exact thing a fixed lane offset
        ignores, and the reason the car used to clip the kerb on the way
        into a bend.

    lo (rule 2): the furthest LEFT the car can sit without any part of it
        crossing the centreline. On a one-way carriageway there is no
        oncoming lane, so the bound falls back to the pavement edge.
    """
    from shapely.geometry import Point
    from shapely.prepared import prep
    # Cache the eroded pavement on the network: it never changes, and both
    # the erosion (a buffer over the whole map) and the repeated contains()
    # queries are far too slow to redo on every route rebuild - that cost
    # showed up as a 0.3 s frame hitch, which moved the car ~9 m in a
    # single physics step and read as a teleport.
    safe = getattr(network, "_raceline_safe", None)
    if safe is None:
        inset = (config.CAR_WIDTH / 2.0 + config.ROAD_EDGE_TOLERANCE_M) * PPPM
        safe = network.get_paved_polygon().buffer(-inset)
        network._raceline_safe = safe
        network._raceline_safe_prep = prep(safe)
    safe_prep = network._raceline_safe_prep

    def reach(px, py, nx, ny, sign):
        """Bisect outward along +-normal for the last on-pavement offset."""
        # None means "no usable measurement here", NOT "no room". This
        # happens routinely at a route's ends: a dead-end node sits at the
        # very tip of the carriageway, and eroding by half a car width
        # pulls the end cap back behind the first stations. Reporting 0
        # there collapsed the corridor to zero width and pinned the line to
        # the centreline for the first few metres, so it then had to jump
        # ~2 m sideways to rejoin the lane - a kink of apparent radius
        # 1.9 m, tighter than the car can steer, at the start of a route.
        if not safe_prep.contains(Point(px, py)):
            return None
        good, bad = 0.0, 8.0
        for _ in range(9):                    # ~0.015 m precision
            mid = 0.5 * (good + bad)
            q = Point(px + nx * sign * mid * PPPM, py + ny * sign * mid * PPPM)
            if safe_prep.contains(q):
                good = mid
            else:
                bad = mid
        return good

    centre_limit = config.CAR_WIDTH / 2.0 + config.LANE_CENTRE_MARGIN_M

    # Probe the pavement only every `stride` stations and interpolate
    # between: road width changes slowly, but each probe costs a bisection
    # (~18 point-in-polygon tests), and doing that at every station of a
    # dense line dominated the whole route rebuild.
    n = len(P)
    stride = max(1, int(CORRIDOR_PROBE_M / max(1e-6, SAMPLE_M)))
    knots = list(range(0, n, stride))
    if knots[-1] != n - 1:
        knots.append(n - 1)

    probe = {}
    for i in knots:
        px, py = P[i]
        nx, ny = N[i]
        probe[i] = (reach(px, py, nx, ny, +1.0), reach(px, py, nx, ny, -1.0))

    def probed(i, k, fallback):
        v = probe[i][k]
        return fallback if v is None else v

    lo, hi = [], []
    for i in range(n):
        a = knots[max(0, min(len(knots) - 2, (i // stride)))]
        b = knots[min(len(knots) - 1, knots.index(a) + 1)]
        t = 0.0 if b == a else (i - a) / (b - a)
        t = max(0.0, min(1.0, t))
        # Cap by the CARRIAGEWAY, not by whatever tarmac happens to be
        # reachable along the normal. At a T-junction the perpendicular
        # runs straight down the crossing road, so an uncapped probe
        # reports several metres of "corridor" out into the intersection
        # and the optimiser happily routes the line through it - which is
        # both illegal and, being a huge lateral excursion, far tighter
        # than the car can steer.
        oneway_i, width_i = station_props[i]
        lane_cap = config.kerb_offset_m(width_i)
        # Inside a junction the carriageway cap does not apply: the paved
        # area is the whole intersection, and a left-turner NEEDS that
        # outside room to get around the centre dot rather than cutting
        # across it. Capping to a single road's width here (while also
        # relaxing the inside bound) had it exactly inverted - it squeezed
        # the car towards the middle, the one place it must not go.
        at_junction = junction_nodes is not None and junction_nodes[i] is not None
        if at_junction:
            lane_cap += JUNCTION_EXTRA_ROOM_M
        # Take the MINIMUM of the bracketing probes, never the average: an
        # interpolated bound may not be a real one, and overstating the
        # corridor by even a few centimetres puts a wheel off the pavement.
        # Where a probe gave no reading, fall back to the nominal
        # carriageway - the probe refines the lane geometry, it is not the
        # only source of it.
        right = min(probed(a, 0, lane_cap), probed(b, 0, lane_cap), lane_cap)
        left = min(probed(a, 1, lane_cap), probed(b, 1, lane_cap), lane_cap)
        h = right
        # One-way roads have no oncoming lane, so the centreline bound does
        # not apply; everywhere else it does.
        l = -left if oneway_i else centre_limit

        # Keep the junction centre on our LEFT. The node's lateral position
        # relative to this station is t (positive = node lies to the
        # right); the car sits at lateral o, so the node is on our left
        # exactly when o > t. Add half the car's width plus a margin so it
        # is the whole body that clears it, not just the centreline.
        if junction_nodes is not None and junction_nodes[i] is not None:
            qx, qy = junction_nodes[i]
            t = ((qx - P[i][0]) * nx + (qy - P[i][1]) * ny) / PPPM
            l = max(l, t + config.CAR_WIDTH / 2.0 + config.LANE_CENTRE_MARGIN_M)
        if l > h:                             # degenerate / very narrow
            l = h = max(0.0, min(l, h))
        lo.append(l)
        hi.append(h)
    return lo, hi


def solve_line(network, rounded, route_seg_idx, ds=SAMPLE_M,
               base_offset: float | None = None,
               merge_from_m: float | None = None,
               merge_s0: float = 0.0, merge_s1: float = 0.0):
    """The fastest legal line for a route, as a dense uniform polyline.

    `base_offset` overrides the nominal lane offset used to pin straight
    sections (see min_curvature_offsets); default is the normal right-lane
    centre. The nav passes the car's spawn lateral position here so the
    car holds its initial line instead of re-centering (docs §1 variant).

    Merge-right blend (docs §1 variant on multi-lane roads): with
    `merge_from_m` given, stations before `merge_s0` settle at
    `merge_from_m` (the spawn lane), stations from `merge_s1` on at
    `base_offset`, and between them a smoothstep - the human-like "change
    lanes right first, then park" instead of holding the overtaking lane
    all the way to the kerb.

    Returns (points, normals, offsets, cum) sampled every ~ds metres.

    Sampling uniformly and densely is not cosmetic. The reference line is
    a polyline, and curvature is measured over a fixed 1 m window: if the
    vertices are ~1 m apart and the lateral offset shifts appreciably
    between them, each vertex reads as a kink whose apparent radius is far
    tighter than the line really is - tight enough to fall below the car's
    3.46 m mechanical minimum and make the speed profile crawl. Dense
    uniform stations keep every vertex-to-vertex bend small.
    """
    P, S, total = _resample(rounded, ds)
    base = (base_offset if base_offset is not None else
            min(config.LANE_OFFSET_DEFAULT_M,
                config.kerb_offset_m(_min_width(network, route_seg_idx))))
    if merge_from_m is not None and abs(merge_from_m - base) > 1e-6 \
            and merge_s1 > merge_s0:
        m0, m1 = merge_s0, merge_s1
        span = max(1e-6, m1 - m0)
        o_end = base          # the settled scalar (resolved above)
        prof: list[float] = []
        for s_ in S:
            if s_ <= m0:
                prof.append(merge_from_m)
            elif s_ >= m1:
                prof.append(o_end)
            else:
                t = (s_ - m0) / span
                t = t * t * (3.0 - 2.0 * t)
                prof.append(merge_from_m + (o_end - merge_from_m) * t)
        base = prof
    if total < 4.0 or len(P) < 5:
        N, _ = _normals_and_curvature(P, ds)
        return P, N, [base] * len(P), S

    N, K = _normals_and_curvature(P, ds)
    props = _station_segments(network, route_seg_idx, P)
    junction = _junction_node_per_station(network, P)
    lo, hi = legal_corridor(network, P, N, props, junction)
    # A straight route needs no curvature solving: a constant offset has
    # ZERO curvature, which is already the optimum. Running the solver
    # there would SMOOTH a per-station base profile - it minimises the
    # profile's own second derivative, stretching the merge blend from its
    # planned ~35 m zone to ~180 m and turning a brisk lane change into a
    # crawl (measured: 8.35->5.25 over [75,110] came back as 7.7@45,
    # 6.2@110, still converging at 120). The solver only earns its keep
    # where the road actually curves.
    if max((abs(k) for k in K), default=0.0) < 1e-4:
        o = [max(lo[i], min(hi[i],
                            _base_at(base, i) if _base_at(base, i) is not None
                            else 0.5 * (lo[i] + hi[i]))) for i in range(len(K))]
    else:
        o = min_curvature_offsets(K, lo, hi, ds, base=base)
    return P, N, o, S


def points_from_offsets(P, N, offsets):
    """Lay offsets onto their stations -> the driven polyline (world px)."""
    return [(P[i][0] + N[i][0] * offsets[i] * PPPM,
             P[i][1] + N[i][1] * offsets[i] * PPPM)
            for i in range(len(P))]


def _min_width(network, route_seg_idx):
    return min((network.segments[i].width for i in route_seg_idx), default=7.0)


def _station_segments(network, route_seg_idx, P):
    """Nearest route segment for each station -> (oneway, width) pairs."""
    segs = [network.segments[i] for i in route_seg_idx] or list(network.segments)
    out = []
    for px, py in P:
        best, bd = None, float("inf")
        for sg in segs:
            dx, dy = sg.x2 - sg.x1, sg.y2 - sg.y1
            L2 = dx * dx + dy * dy
            if L2 < 1e-9:
                continue
            t = max(0.0, min(1.0, ((px - sg.x1) * dx + (py - sg.y1) * dy) / L2))
            d = math.hypot(px - (sg.x1 + t * dx), py - (sg.y1 + t * dy))
            if d < bd:
                bd, best = d, sg
        out.append((bool(best.oneway), best.width) if best else (False, 7.0))
    return out
