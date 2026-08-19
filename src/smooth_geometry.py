# Smoothed geometry — the §10 pipeline
#
# "The graph is sacred; the curve is a function of the graph"
# (docs/TURN_REWORK_PLAN.md §10).
#
# The OSM node set and way topology are never touched. Each merged road
# line (a chain of segments through degree-2 nodes, as produced by
# RoadNetwork._merge_and_round_lines) becomes a centripetal Catmull-Rom
# spline through its ORIGINAL nodes (interpolating: the curve passes
# exactly through every node; C1: no kinks). At a real junction
# (degree >= 3) the spline ends with a well-defined end tangent (the
# last-chord direction), and the usual 6 m circular fillet arc connects
# the approach tangent to the exit tangent — tangent-continuous with the
# splines, because the spline's end tangent IS the chord direction.
#
# Everything downstream consumes the SAME smoothed geometry:
#
#   paved polygon      = buffer of the smoothed lines + fillet patches
#   centerline dashes  = the smoothed lines
#   lane markings      = offsets of the smoothed lines
#   driving reference  = sub-curves of the smoothed lines + the same
#   line (BicycleNav)      fillet arcs, lane-offset
#
# so what is drawn is exactly what is driven and what the on-road check
# tests. The dense resampled polylines are evaluation caches of the curve
# functions (like the arc-length table) — they define no new geometry and
# touch no graph node.
#
# Curvature is ALWAYS measured from geometry (central differences of
# point_at), never from a per-sample table: the first SmoothCurve draft
# stored a per-sample kappa table that was corrupted at every piece
# junction (a duplicate sample emitted ds = 0 and a heading of 0, spiking
# kappa to ~16 1/m). The duplicate samples are gone from the table AND
# curvature_at is geometric, so both failure modes are impossible.

from __future__ import annotations

import math

from . import config
from .road_network import RoadNetwork, _merge_and_round_lines


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
    """A centripetal Catmull-Rom spline through a list of 2D control
    points (world PIXELS), arc-length parameterized in METRES.

    Interpolating (the curve passes exactly through every control point —
    faithful to the OSM data) and C1-continuous (no kinks). `point_at`,
    `heading_at` and `curvature_at` are smooth functions of arc length s,
    evaluated against a dense arc-length lookup table built once in the
    constructor (an evaluation cache, not new geometry).

    heading: degrees, 0 = north (+y), positive = clockwise (east), matching
    the rest of the codebase (forward = (sin h, cos h)).
    curvature: 1/m, signed, positive = right turn (heading increasing).
    Measured from geometry (central differences of point_at), never from a
    per-sample table (see module docstring).
    """

    def __init__(self, pts: list[tuple[float, float]], pppm: float = config.PIXELS_PER_METER,
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
            return

        # Centripetal knot values: alpha = 0.5.
        self._t = [0.0]
        for i in range(1, n):
            d = math.hypot(self.pts[i][0] - self.pts[i - 1][0],
                           self.pts[i][1] - self.pts[i - 1][1])
            self._t.append(self._t[-1] + max(d, 1e-6) ** 0.5)

        # Dense arc-length table. Each piece is sampled with a count
        # proportional to its chord length (tight/long pieces get more
        # samples) so the table is uniform in arc length to within
        # ~sample_m. Piece i emits k = 0..steps-1 and the final endpoint
        # is emitted exactly once — the shared piece junction is NOT
        # emitted twice (the duplicate sample of the first draft corrupted
        # its per-sample curvature table).
        s_tab: list[float] = []
        x_tab: list[float] = []
        y_tab: list[float] = []
        hdg_tab: list[float] = []
        # Endpoint tangents: the curve leaves the first node toward the
        # second node and arrives at the last node from the second-to-last
        # (§10: "the spline ends at the junction node with a well-defined
        # end tangent, direction last-but-one -> last node"). We enforce
        # this by giving the duplicated endpoint knot a spacing that makes
        # the centripetal tangent at the endpoint equal to the adjacent-
        # segment direction (a standard clamped Catmull-Rom endpoint).
        cum = 0.0
        prev: tuple[float, float] | None = None
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
            for k in range(steps):
                t = t1 + (t2 - t1) * k / steps
                x, y = _cr_point(p0, p1, p2, p3, t0, t1, t2, t3, t)
                if prev is None:
                    # First sample: heading = the clamped start tangent
                    # (the p1->p2 chord direction).
                    hdg = math.atan2(p2[0] - p1[0], p2[1] - p1[1])
                else:
                    ds = math.hypot(x - prev[0], y - prev[1]) / pppm
                    cum += ds
                    hdg = math.atan2(x - prev[0], y - prev[1])
                s_tab.append(cum)
                x_tab.append(x)
                y_tab.append(y)
                hdg_tab.append(hdg)
                prev = (x, y)
        # Final endpoint (t = end of the last piece), emitted once.
        x, y = _cr_point(p0, p1, p2, p3, t0, t1, t2, t3, t2)
        ds = math.hypot(x - prev[0], y - prev[1]) / pppm
        cum += ds
        hdg = math.atan2(x - prev[0], y - prev[1])
        s_tab.append(cum)
        x_tab.append(x)
        y_tab.append(y)
        hdg_tab.append(hdg)

        self._s = s_tab
        self._x = x_tab
        self._y = y_tab
        self._hdg = hdg_tab
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
        """Signed curvature (1/m) at arc length s (positive = right turn).

        Geometry-based: central differences of point_at over a fixed
        window. Never a per-sample table (see module docstring)."""
        if self.total <= 0:
            return 0.0
        h = max(1.0, self.total * 0.01)
        s1 = max(0.0, s - h)
        s2 = min(self.total, s + h)
        if s2 - s1 < 1e-3:
            return 0.0
        # Tangent heading at each window end from a local chord (0 =
        # north, positive = clockwise, like the rest of the codebase).
        e = min(0.5, (s2 - s1) / 4.0)
        ax, ay = self.point_at(max(0.0, s1 - e))
        bx, by = self.point_at(min(self.total, s1 + e))
        h1 = math.atan2(bx - ax, by - ay)
        cx, cy = self.point_at(max(0.0, s2 - e))
        dx, dy = self.point_at(min(self.total, s2 + e))
        h2 = math.atan2(dx - cx, dy - cy)
        dh = (h2 - h1 + math.pi) % (2 * math.pi) - math.pi
        return dh / (s2 - s1)


def resample_curve(curve: SmoothCurve, step_m: float = 0.5) -> list[tuple[float, float]]:
    """Dense polyline (world pixels) of the curve, resampled every
    step_m of arc length. A render/eval cache of the curve function —
    it defines no new geometry."""
    if curve.total <= 0:
        return [curve.point_at(0.0)]
    n = max(2, int(curve.total / step_m) + 1)
    pts = [curve.point_at(curve.total * i / (n - 1)) for i in range(n)]
    return pts


def corner_fillet(prev_pt: tuple[float, float], vertex: tuple[float, float],
                  next_pt: tuple[float, float], radius: float,
                  arc_steps: int = 10):
    """Circular fillet of `radius` (pixels) tangent to both edges of the
    corner at `vertex` — the exact math the old `_round_polyline_corners`
    used (now the single shared implementation).

    Returns (t1, t2, arc_pts, actual_radius, tangent_dist) where t1 lies
    on the incoming edge, t2 on the outgoing edge, arc_pts is the arc
    from t1 to t2 (arc_steps+1 points, endpoints included), and
    actual_radius/tangent_dist account for the cap that keeps the fillet
    from eating more than half of either adjoining edge (short stubs).
    Returns None if there is no real corner (straight-through,
    doubled-back, or degenerate edges).
    """
    px, py = prev_pt
    vx, vy = vertex
    nx, ny = next_pt

    ax, ay = px - vx, py - vy
    bx, by = nx - vx, ny - vy
    a_len = math.hypot(ax, ay)
    b_len = math.hypot(bx, by)
    if a_len < 1e-9 or b_len < 1e-9:
        return None
    ax, ay = ax / a_len, ay / a_len
    bx, by = bx / b_len, by / b_len

    # Angle between the two edges (both pointing away from the vertex).
    dot = max(-1.0, min(1.0, ax * bx + ay * by))
    gap = math.acos(dot)
    if gap < 1e-6 or gap > math.pi - 1e-6:
        # Straight-through (or fully doubled-back) - no real corner.
        return None

    half_gap = gap / 2
    tangent_dist = radius / math.tan(half_gap)
    # Cap the tangent distance so it never eats more than half of either
    # adjoining edge (avoids self-overlap on short segments).
    tangent_dist = min(tangent_dist, a_len / 2, b_len / 2)
    actual_radius = tangent_dist * math.tan(half_gap)

    center_dist = actual_radius / math.sin(half_gap)
    bis_x, bis_y = ax + bx, ay + by
    bis_len = math.hypot(bis_x, bis_y)
    if bis_len < 1e-9:
        return None
    bis_x, bis_y = bis_x / bis_len, bis_y / bis_len

    center_x = vx + center_dist * bis_x
    center_y = vy + center_dist * bis_y
    t1 = (vx + tangent_dist * ax, vy + tangent_dist * ay)
    t2 = (vx + tangent_dist * bx, vy + tangent_dist * by)

    angle_t1 = math.atan2(t1[1] - center_y, t1[0] - center_x)
    angle_t2 = math.atan2(t2[1] - center_y, t2[0] - center_x)

    # The correct arc sweep is always the EXTERIOR turn angle (pi - gap) -
    # a small, sharp corner (gap near 0) needs a wide near-180 deg arc, a
    # nearly-straight corner (gap near pi) needs a tiny arc. Of the two
    # possible *rotation directions* around the circle from t1 to t2, pick
    # whichever one's size actually matches that value. t1 must ALWAYS be
    # emitted before t2 - t1 lies on the incoming edge, t2 on the outgoing
    # edge, so swapping their order self-intersects the line.
    fwd_sweep = (angle_t2 - angle_t1) % (2 * math.pi)
    expected_sweep = math.pi - gap
    if abs(fwd_sweep - expected_sweep) < abs((2 * math.pi - fwd_sweep) - expected_sweep):
        direction, sweep = 1, fwd_sweep
    else:
        direction, sweep = -1, (2 * math.pi) - fwd_sweep

    arc = [t1]
    for s in range(1, arc_steps + 1):
        a = angle_t1 + direction * sweep * (s / arc_steps)
        arc.append((center_x + actual_radius * math.cos(a),
                    center_y + actual_radius * math.sin(a)))
    return t1, t2, arc, actual_radius, tangent_dist


class SmoothedNetwork:
    """The §10 smoothed geometry of one RoadNetwork, built once and
    cached.

    - `lines`: the merged road lines (same group key and same merged
      polylines as _merge_and_round_lines), each with its SmoothCurve
      (through the original nodes) and a dense resampled polyline.
    - `junction_fillets`: for every degree>=3 node, the circular fillet
      arcs between adjacent spokes (same filters and radii as the old
      _build_multiway_junction_fillets) — the §10 junction connections,
      tangent-continuous with the line splines.
    - `segment_curve`: (seg_idx, direction) -> (curve, s_start, s_end)
      for every segment in both travel directions, so the driving model
      can take the exact sub-curve of a segment's line.
    """

    MIN_GAP_DEG = 15.0
    MAX_GAP_DEG = 155.0

    def __init__(self, network: RoadNetwork, resample_m: float = 0.5):
        self.network = network
        self.pppm = config.PIXELS_PER_METER
        corner_radius_px = config.ROAD_CORNER_RADIUS_M * self.pppm

        # ---- per merged line: spline + resampled polyline ----
        groups = _merge_and_round_lines(network)
        self.lines: list[dict] = []
        for (highway, width), coords_list in groups.items():
            for coords in coords_list:
                curve = SmoothCurve(coords, pppm=self.pppm)
                self.lines.append({
                    "highway": highway,
                    "width": width,
                    "coords": coords,          # the original merged nodes (px)
                    "curve": curve,
                    "resampled": resample_curve(curve, resample_m),
                })

        # ---- segment -> (curve, s range) in both directions ----
        # A segment is one chord of exactly one merged line (linemerge
        # only joins through degree-2 nodes, and a line's interior nodes
        # are degree-2 by construction). Find that line by endpoint match
        # and the node arc-length positions on its spline. (Keyed by
        # segment INDEX, not seg.id: in OSM data every chord of a way
        # shares the way's id, so ids are not unique per segment.)
        self.segment_curve: dict[tuple[int, bool], tuple] = {}
        for idx, seg in enumerate(network.segments):
            for forward in (True, False):
                a = seg.start_node if forward else seg.end_node
                b = seg.end_node if forward else seg.start_node
                pa, pb = network.nodes[a], network.nodes[b]
                hit = None
                for line in self.lines:
                    cs = line["coords"]
                    if len(cs) < 2:
                        continue
                    if (cs[0] == pa and cs[1] == pb) or (cs[0] == pb and cs[1] == pa):
                        hit = (line, 0.0, line["curve"].total)
                        break
                    for i in range(len(cs) - 1):
                        if (cs[i] == pa and cs[i + 1] == pb) or \
                                (cs[i] == pb and cs[i + 1] == pa):
                            # Arc length of each node on the spline: the
                            # spline passes exactly through the nodes, so
                            # the table sample nearest the node IS the node.
                            sa = self._node_s(line["curve"], pa)
                            sb = self._node_s(line["curve"], pb)
                            hit = (line, min(sa, sb), max(sa, sb))
                            break
                    if hit:
                        break
                if hit is None:
                    # Degenerate (zero-length or unmatched) segment: a
                    # trivial straight curve so callers always get a curve.
                    hit = ({"curve": SmoothCurve([pa, pb], pppm=self.pppm)},
                           0.0, 0.0)
                self.segment_curve[(idx, forward)] = hit

        # ---- junction fillets (degree >= 3) ----
        self.junction_fillets: list[dict] = []
        for node_id, connected in network.node_connections.items():
            if len(connected) < 3:
                continue  # degree-2 bends are inside the line splines
            node_xy = network.nodes.get(node_id)
            if node_xy is None:
                continue
            node_x, node_y = node_xy

            spokes = []
            for seg_idx in connected:
                seg = network.segments[seg_idx]
                if seg.start_node == node_id:
                    away_dx, away_dy = seg.x2 - seg.x1, seg.y2 - seg.y1
                else:
                    away_dx, away_dy = seg.x1 - seg.x2, seg.y1 - seg.y2
                length = math.hypot(away_dx, away_dy)
                if length < 1e-6:
                    continue
                spokes.append((away_dx / length, away_dy / length, seg, seg_idx))
            if len(spokes) < 2:
                continue
            spokes.sort(key=lambda s: math.atan2(s[1], s[0]))

            n = len(spokes)
            for i in range(n):
                ax, ay, seg_a, seg_a_id = spokes[i]
                bx, by, seg_b, seg_b_id = spokes[(i + 1) % n]
                gap = math.acos(max(-1.0, min(1.0, ax * bx + ay * by)))
                gap_deg = math.degrees(gap)
                if gap_deg < self.MIN_GAP_DEG or gap_deg > self.MAX_GAP_DEG:
                    continue
                half_w_px = (min(seg_a.width, seg_b.width) / 2) * self.pppm
                reach = corner_radius_px * 3 + half_w_px
                fillet = corner_fillet(
                    (node_x + reach * ax, node_y + reach * ay),
                    (node_x, node_y),
                    (node_x + reach * bx, node_y + reach * by),
                    corner_radius_px,
                )
                if fillet is None:
                    continue
                t1, t2, arc, actual_radius, tangent_dist = fillet
                self.junction_fillets.append({
                    "node": node_id,
                    "seg_a": seg_a_id,
                    "seg_b": seg_b_id,
                    "t1": t1,
                    "t2": t2,
                    "arc": arc,
                    "radius_px": actual_radius,
                })

    @staticmethod
    def _node_s(curve: SmoothCurve, node: tuple[float, float]) -> float:
        """Arc length at which the spline passes through `node` (it does,
        exactly — the spline is interpolating)."""
        best_s = 0.0
        best_d2 = float("inf")
        n = len(curve._s)
        for i in range(0, n, max(1, n // 2000)):
            x, y = curve._x[i], curve._y[i]
            d2 = (x - node[0]) ** 2 + (y - node[1]) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = curve._s[i]
        # Refine locally.
        lo = max(0.0, best_s - 2.0)
        hi = min(curve.total, best_s + 2.0)
        for k in range(41):
            s = lo + (hi - lo) * k / 40
            x, y = curve.point_at(s)
            d2 = (x - node[0]) ** 2 + (y - node[1]) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = s
        return best_s


# Convenience accessor: one SmoothedNetwork per RoadNetwork (cached).
def smoothed_network(network: RoadNetwork) -> SmoothedNetwork:
    cache = getattr(network, "_smoothed_cache", None)
    if cache is None:
        cache = SmoothedNetwork(network)
        network._smoothed_cache = cache
    return cache
