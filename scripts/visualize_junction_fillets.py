#!/usr/bin/env python3
"""Visualize junction connections: CURRENT vs PROPOSED (spline-generated).

CURRENT (today's code):
  - rendering / paved area: circular fillet, R = 6 m
    (src/road_network.py: _round_polyline_corners +
     _build_multiway_junction_fillets - the ACTUAL functions are reused)
  - driving line: the same circular fillet, lane-offset, then run through
    the project's centripetal Catmull-Rom SmoothCurve (bicycle_nav.py) -
    exactly what the bicycle model follows today.

PROPOSED (spline-generated junction connections, §10 idea):
  - the junction connection itself is a cubic Bezier between the SAME
    tangent points the circular fillet uses (same footprint, same paved
    area), so the whole route - approach, connection, exit - is one
    smooth spline family; the driving line is the same SmoothCurve
    through the Bezier-connection points.

Outputs one PNG per junction type into /tmp/junction_fillets/
(corner_90, t_junction, y_junction, crossroads, sliver). Each PNG:
  top    paved area + centerlines + the driven route (thick yellow)
  bottom curvature kappa(s) along the driven route - the dynamic
         difference (the car limit R = 3.46 m is marked in red)
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, Polygon
from shapely.ops import linemerge, unary_union

from src import config
from src.bicycle_nav import PPPM, SmoothCurve, _offset_polyline_right
from src.road_network import (
    RoadNetwork,
    RoadSegment,
    _build_multiway_junction_fillets,
    _round_polyline_corners,
)

R = config.ROAD_CORNER_RADIUS_M  # 6.0 m
W = 7.0  # m, two-way width used by the test map
LANE_OFFSET = 1.75  # m, right-lane offset for 7 m roads (bicycle_nav)

# Mechanical minimum turning radius (AGENTS.md hard rule):
# WHEELBASE / tan(MAX_STEER) = 2.7 / tan(38 deg)
MIN_RADIUS_M = 2.7 / math.tan(math.radians(38))  # ~3.46 m


# ----------------------------------------------------------------------
# Bezier junction connection: same tangent points as the circular fillet,
# cubic Bezier between them (control reach k = R * 4/3 * tan(phi/4), the
# standard cubic approximation of a circular arc - peak curvature matches
# the arc's 1/R).
# ----------------------------------------------------------------------

def _corner_tangents(px, py, vx, vy, nx, ny, radius):
    """Replicates _round_polyline_corners' tangent math exactly."""
    ax, ay = px - vx, py - vy
    bx, by = nx - vx, ny - vy
    a_len = math.hypot(ax, ay)
    b_len = math.hypot(bx, by)
    if a_len < 1e-9 or b_len < 1e-9:
        return None
    ax, ay = ax / a_len, ay / a_len
    bx, by = bx / b_len, by / b_len
    dot = max(-1.0, min(1.0, ax * bx + ay * by))
    gap = math.acos(dot)
    if gap < 1e-6 or gap > math.pi - 1e-6:
        return None
    half_gap = gap / 2
    T = min(radius / math.tan(half_gap), a_len / 2, b_len / 2)
    actual_radius = T * math.tan(half_gap)
    t1 = (vx + T * ax, vy + T * ay)
    t2 = (vx + T * bx, vy + T * by)
    # travel directions (unit): at t1 toward the vertex, at t2 away from it
    dir_in = (-ax, -ay)
    dir_out = (bx, by)
    return t1, t2, dir_in, dir_out, actual_radius, T


def _bezier_point(p0, p1, p2, p3, t):
    u = 1 - t
    return (
        u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0],
        u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1],
    )


def _bezier_fillet(coords, radius, samples=32):
    """Sibling of _round_polyline_corners: Bezier connection between the
    same tangent points the circular fillet uses."""
    result = [coords[0]]
    for i in range(1, len(coords) - 1):
        px, py = coords[i - 1]
        vx, vy = coords[i]
        nx, ny = coords[i + 1]
        ct = _corner_tangents(px, py, vx, vy, nx, ny, radius)
        if ct is None:
            result.append((vx, vy))
            continue
        t1, t2, dir_in, dir_out, actual_radius, _T = ct
        phi = math.acos(max(-1.0, min(1.0,
                                      -(dir_in[0] * dir_out[0] + dir_in[1] * dir_out[1]))))
        k = actual_radius * (4.0 / 3.0) * math.tan(phi / 4.0)
        p0, p3 = t1, t2
        p1 = (t1[0] + k * dir_in[0], t1[1] + k * dir_in[1])
        # p2 lies BEHIND p3, opposite the travel direction: the curve
        # arrives at p3 heading toward dir_out.
        p2 = (t2[0] - k * dir_out[0], t2[1] - k * dir_out[1])
        result.append(p0)
        for s in range(1, samples + 1):
            result.append(_bezier_point(p0, p1, p2, p3, s / samples))
    result.append(coords[-1])
    return result


def _bezier_junction_fillets(network: RoadNetwork, pppm: float, samples=32):
    """Sibling of _build_multiway_junction_fillets: same virtual 3-point
    lines and filters, Bezier connection instead of the circular arc."""
    MIN_GAP_DEG = 15.0
    MAX_GAP_DEG = 155.0
    corner_radius = config.ROAD_CORNER_RADIUS_M * pppm

    extras = []
    for node_id, connected in network.node_connections.items():
        if len(connected) < 3:
            continue
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
            angle = math.atan2(away_dy, away_dx)
            spokes.append((angle, away_dx / length, away_dy / length, seg))
        if len(spokes) < 2:
            continue
        spokes.sort(key=lambda s: s[0])

        n = len(spokes)
        for i in range(n):
            _, ax, ay, seg_a = spokes[i]
            _, bx, by, seg_b = spokes[(i + 1) % n]
            gap_deg = math.degrees(math.acos(max(-1.0, min(1.0, ax * bx + ay * by))))
            if gap_deg < MIN_GAP_DEG or gap_deg > MAX_GAP_DEG:
                continue
            half_w = (min(seg_a.width, seg_b.width) / 2) * pppm
            reach = corner_radius * 3 + half_w
            virtual_line = [
                (node_x + reach * ax, node_y + reach * ay),
                (node_x, node_y),
                (node_x + reach * bx, node_y + reach * by),
            ]
            smooth = _bezier_fillet(virtual_line, corner_radius, samples)
            buffered = LineString(smooth).buffer(
                half_w, cap_style="round", join_style="round", resolution=8
            )
            color = config.ROAD_TYPES.get(seg_a.highway, {}).get("color", (150, 150, 150))
            polys = buffered.geoms if hasattr(buffered, "geoms") else [buffered]
            exteriors = [(list(p.exterior.coords), [list(r.coords) for r in p.interiors])
                         for p in polys if not p.is_empty]
            extras.append((color, exteriors))
    return extras


# ----------------------------------------------------------------------
# Tiny synthetic networks (test-map junction layouts, in metres)
# ----------------------------------------------------------------------

def make_network(lines: list[list[tuple[float, float]]], width: float = W) -> RoadNetwork:
    nodes: dict[str, tuple[float, float]] = {}
    segments: list[RoadSegment] = []
    node_connections: dict[str, list[int]] = {}
    node_degree: dict[str, int] = {}

    def node_id(pt):
        key = (round(pt[0], 6), round(pt[1], 6))
        for k, v in nodes.items():
            if (round(v[0], 6), round(v[1], 6)) == key:
                return k
        k = f"n{len(nodes)}"
        nodes[k] = pt
        return k

    seg_id = 0
    for line in lines:
        for i in range(len(line) - 1):
            n1, n2 = node_id(line[i]), node_id(line[i + 1])
            x1, y1 = nodes[n1]
            x2, y2 = nodes[n2]
            segments.append(RoadSegment(
                id=seg_id, x1=x1, y1=y1, x2=x2, y2=y2,
                highway="residential", oneway=False, width=width,
                start_node=n1, end_node=n2,
                length=math.hypot(x2 - x1, y2 - y1),
            ))
            for nid in (n1, n2):
                node_connections.setdefault(nid, []).append(seg_id)
                node_degree[nid] = node_degree.get(nid, 0) + 1
            seg_id += 1

    xs = [p[0] for p in nodes.values()]
    ys = [p[1] for p in nodes.values()]
    return RoadNetwork(
        nodes=nodes, segments=segments,
        origin_lat=0.0, origin_lon=0.0,
        world_width=max(xs) - min(xs), world_height=max(ys) - min(ys),
        node_connections=node_connections, node_degree=node_degree,
    )


def case_lines(name: str):
    """Raw chord polylines per junction type (junction at the origin)."""
    if name == "corner_90":
        return [[(0.0, 120.0), (0.0, 0.0), (120.0, 0.0)]]
    if name == "t_junction":
        return [[(0.0, 120.0), (0.0, 0.0)],
                [(-120.0, 0.0), (0.0, 0.0), (120.0, 0.0)]]
    if name == "y_junction":
        return [[(0.0, 180.0), (0.0, 0.0)],
                [(0.0, 0.0), (-150.0, -170.0)],
                [(0.0, 0.0), (150.0, -170.0)]]
    if name == "crossroads":
        return [[(0.0, 180.0), (0.0, 0.0)], [(0.0, 0.0), (0.0, -170.0)],
                [(-170.0, 0.0), (0.0, 0.0)], [(0.0, 0.0), (170.0, 0.0)]]
    if name == "sliver":
        return [[(-0.73, 4.16), (0.0, 0.0)],
                [(0.0, 0.0), (0.74, -4.93)],
                [(-36.2, -5.4), (0.0, 0.0)],
                [(0.0, 0.0), (19.6, 2.7)]]
    raise KeyError(name)


def case_route(name: str):
    """One representative driven route (chords) through the junction."""
    if name == "corner_90":
        return [(0.0, 120.0), (0.0, 0.0), (120.0, 0.0)]
    if name == "t_junction":
        return [(0.0, 120.0), (0.0, 0.0), (120.0, 0.0)]
    if name == "y_junction":
        return [(0.0, 180.0), (0.0, 0.0), (-150.0, -170.0)]
    if name == "crossroads":
        return [(0.0, 180.0), (0.0, 0.0), (170.0, 0.0)]
    if name == "sliver":
        return [(-0.73, 4.16), (0.0, 0.0), (-36.2, -5.4)]
    raise KeyError(name)


def case_title(name: str):
    return {
        "corner_90": "90° corner (degree 2) - like corner_right_entry",
        "t_junction": "T-junction (degree 3) - like tjunction_from_top",
        "y_junction": "Y-intersection (degree 3) - like y_from_stem",
        "crossroads": "4-way crossroads (degree 4) - like crossroads_from_north",
        "sliver": "Sliver junction (degree 4, 4.2 m approach) - like sliver_approach",
    }[name]


# ----------------------------------------------------------------------
# Build both variants of a case
# ----------------------------------------------------------------------

def build_case(name: str) -> dict:
    lines = case_lines(name)
    route = case_route(name)
    net = make_network(lines)

    # ---- CURRENT: the actual code path -------------------------------
    merged = linemerge([LineString(l) for l in lines])
    merged_lines = merged.geoms if hasattr(merged, "geoms") else [merged]
    cur_center = [_round_polyline_corners(list(l.coords), R) for l in merged_lines]
    cur_paved_parts = [
        LineString(c).buffer(W / 2, cap_style="round", join_style="round", resolution=8)
        for c in cur_center
    ]
    for _color, exteriors in _build_multiway_junction_fillets(net, 1.0):
        for ext, holes in exteriors:
            cur_paved_parts.append(Polygon(ext, holes))
    cur_paved = unary_union(cur_paved_parts)
    # driving line: circular fillet -> lane offset -> Catmull-Rom (game code)
    # (coordinates here are metres, but _offset_polyline_right multiplies by
    # PPPM internally, so divide the offset by PPPM to get a true 1.75 m)
    off = LANE_OFFSET / PPPM
    cur_drive_pts = _offset_polyline_right(_round_polyline_corners(route, R), off)
    cur_drive = SmoothCurve(cur_drive_pts, pppm=1.0)

    # ---- PROPOSED: Bezier junction connections -----------------------
    bez_center = [_bezier_fillet(list(l), R) for l in lines]
    bez_paved_parts = [
        LineString(c).buffer(W / 2, cap_style="round", join_style="round", resolution=8)
        for c in bez_center
    ]
    for _color, exteriors in _bezier_junction_fillets(net, 1.0):
        for ext, holes in exteriors:
            bez_paved_parts.append(Polygon(ext, holes))
    bez_paved = unary_union(bez_paved_parts)
    bez_drive_pts = _offset_polyline_right(_bezier_fillet(route, R), off)
    bez_drive = SmoothCurve(bez_drive_pts, pppm=1.0)

    return dict(
        name=name, net=net,
        cur_center=cur_center, cur_paved=cur_paved, cur_drive=cur_drive,
        bez_center=bez_center, bez_paved=bez_paved, bez_drive=bez_drive,
    )


# ----------------------------------------------------------------------
# Curvature sampling along a SmoothCurve (the game's own curvature)
# ----------------------------------------------------------------------

def sample_curve(curve: SmoothCurve, n: int = 500):
    """Sample position + curvature along the curve.

    NOTE: curvature is computed here from the curve's GEOMETRY (central
    differences of point_at), not from SmoothCurve's own kap table - the
    table contains a known artifact: every spline-piece junction emits a
    duplicate sample (ds = 0, heading = atan2(0,0) = 0), which corrupts
    the curvature of the following sample (spikes up to ~16 1/m at
    control points; the 1 m speed-profile grid mostly misses them, but
    this is a latent bug in src/bicycle_nav.py SmoothCurve). point_at
    itself is correct, so the geometry-based kappa below is the true
    curvature of the line the car actually follows.
    """
    total = curve.total
    s = [total * j / (n - 1) for j in range(n)]
    pts = [curve.point_at(si) for si in s]
    h = max(total / 400.0, 0.05)

    def at(samp):
        samp = min(max(samp, 0.0), total)
        return curve.point_at(samp)

    d = min(h, 0.3)  # step for the tangent estimate

    def heading_at(samp):
        xm, ym = at(samp - d)
        xp, yp = at(samp + d)
        return math.atan2(xp - xm, yp - ym)

    k = []
    for si in s:
        if si - h < 0.0 or si + h > total:
            k.append(0.0)
            continue
        # kappa = change in tangent heading between the two sample
        # points, divided by the arc distance between them. (Each
        # heading is itself a central difference, so on a circle of
        # radius R this reads exactly 1/R.)
        a1 = heading_at(si - h)
        a2 = heading_at(si + h)
        da = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        k.append(abs(da) / (2 * h))
    return s, k, pts


# ----------------------------------------------------------------------
# Drawing
# ----------------------------------------------------------------------

def paved_rings(paved):
    geoms = paved.geoms if hasattr(paved, "geoms") else [paved]
    rings = []
    for g in geoms:
        if g.is_empty:
            continue
        rings.append(list(g.exterior.coords))
        for r in g.interiors:
            rings.append(list(r.coords))
    return rings


def draw_case(case: dict, out_path: str):
    name = case["name"]
    fig, axes = plt.subplots(
        2, 2, figsize=(15.5, 11.5),
        gridspec_kw={"height_ratios": [1.6, 1.0], "hspace": 0.42, "wspace": 0.10},
    )
    ax_top_l, ax_top_r = axes[0]
    ax_bot_l, ax_bot_r = axes[1]
    fig.suptitle(
        f"{case_title(name)}\n"
        f"left: CURRENT (circular fillet R = {R:.1f} m; driving line = fillet + Catmull-Rom, today's code)\n"
        f"right: PROPOSED (junction connection generated as a spline - Bezier between the same tangent points)",
        fontsize=13,
    )

    for ax, paved, center, drive, label in (
        (ax_top_l, case["cur_paved"], case["cur_center"], case["cur_drive"], "CURRENT"),
        (ax_top_r, case["bez_paved"], case["bez_center"], case["bez_drive"], "PROPOSED"),
    ):
        for ring in paved_rings(paved):
            ax.add_patch(MplPolygon(ring, closed=True, facecolor="#888888",
                                    edgecolor="#555555", linewidth=1.0, zorder=1))
        for c in center:
            ax.plot([p[0] for p in c], [p[1] for p in c], color="white",
                    linewidth=1.2, zorder=2, alpha=0.9)
        s, _k, pts = sample_curve(drive, 300)
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#ffcc00",
                linewidth=3.0, zorder=3, solid_capstyle="round")
        for nid, (nx, ny) in case["net"].nodes.items():
            if case["net"].node_degree.get(nid, 0) >= 3:
                ax.plot(nx, ny, "o", color="red", markersize=5, zorder=4)
        ax.set_title(label, fontsize=12,
                     color="#d73c3c" if label == "CURRENT" else "#2e8b57")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

    for ax, drive, label in (
        (ax_bot_l, case["cur_drive"], "CURRENT"),
        (ax_bot_r, case["bez_drive"], "PROPOSED"),
    ):
        s, k, _pts = sample_curve(drive)
        ax.fill_between(s, k, color="#ffcc00", alpha=0.7)
        ax.plot(s, k, color="#b8860b", linewidth=1.0)
        ax.axhline(1.0 / MIN_RADIUS_M, color="red", linestyle="--", linewidth=1.0,
                   label=f"car limit (R = {MIN_RADIUS_M:.2f} m)")
        ax.set_title(f"curvature κ(s) of the driven route - {label}", fontsize=11)
        ax.set_xlabel("distance along route (m)")
        ax.set_ylabel("κ (1/m)")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8, loc="upper right")
        kmax = max(k)
        ax.annotate(f"max κ = {kmax:.3f} 1/m  (R = {1.0 / kmax:.2f} m)",
                    xy=(0.02, 0.95), xycoords="axes fraction", fontsize=9,
                    color="darkred", va="top")

    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f"saved {out_path}")


def main():
    out_dir = "/tmp/junction_fillets"
    os.makedirs(out_dir, exist_ok=True)
    for name in ("corner_90", "t_junction", "y_junction", "crossroads", "sliver"):
        case = build_case(name)
        draw_case(case, os.path.join(out_dir, f"{name}.png"))
        for label, drive in (("current", case["cur_drive"]), ("proposed", case["bez_drive"])):
            s, k, _ = sample_curve(drive)
            kmax = max(k)
            print(f"  {name:12s} {label:9s} max kappa = {kmax:.4f} 1/m "
                  f"(R = {1.0 / kmax:6.2f} m)"
                  + ("  <-- BELOW car minimum!" if kmax > 1.0 / MIN_RADIUS_M else ""))


if __name__ == "__main__":
    main()
