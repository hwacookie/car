#!/usr/bin/env python3
"""Raceline reference dump for the C# port (Phase 2 gate).

Builds the basic test map, solves a battery of routes through every code
path of src/raceline.py, and dumps full-precision JSON so the C# port
(DrivingGame.Sim/Raceline.cs) can be compared against it.

Gate (docs/C_SHARP_PORT_SPEC.md, Phase 2): solve outputs vs this dump at
~1e-9 tolerance. The dump also carries the RAW paved polygon and the
ERODED safe polygon so the C# side can run its solver on IDENTICAL
geometry (NTS vs GEOS buffer equivalence is checked separately with a
looser tolerance).

Usage:  python3 tools/raceline_reference.py [out.json]
Default out: ../driving-game/data/raceline_reference/basic.json
"""

import json
import os
import sys

sys.path.insert(0, ".")

from src import config                      # noqa: E402
from src import raceline                    # noqa: E402
from src.road_network import _round_polyline_corners  # noqa: E402
from src.test_maps import build_basic_test_map        # noqa: E402

PPPM = config.PIXELS_PER_METER
# Must match BicycleNav's corner rounding (src/bicycle_nav.py):
CORNER_RADIUS_M = config.ROAD_CORNER_RADIUS_M   # 6.0
CORNER_ARC_STEPS = 48


def route_seg_indices(network, nodes):
    """Segment index for each consecutive node pair (either direction)."""
    idxs = []
    for a, b in zip(nodes, nodes[1:]):
        found = None
        for i, sg in enumerate(network.segments):
            if (sg.start_node == a and sg.end_node == b) or \
               (sg.start_node == b and sg.end_node == a):
                found = i
                break
        assert found is not None, f"no segment {a} -> {b}"
        idxs.append(found)
    return idxs


# (name, node sequence, solve_line kwargs) — one case per code path of
# interest: straight fast path, degree-2 corners, degree-3+ junctions
# (right / straight / left), one-way corridors, multi-lane + parking
# nominals, width-step diagonals, merge-right blend, long routes.
CASES = [
    ("straight", ["straight_n", "straight_s"], {}),
    ("corner_right", ["cornerR_n", "cornerR_c", "cornerR_w"], {"auto_base": True}),
    ("corner_left", ["cornerL_n", "cornerL_c", "cornerL_e"], {"auto_base": True}),
    # degree-3 junction: right turn keeps the dot on the left, left turn is
    # excluded from that bound (StVO 9(4) voreinander).
    ("tjunc_right", ["tjunc_top", "tjunc_center", "tjunc_w"], {"auto_base": True}),
    ("tjunc_left", ["tjunc_top", "tjunc_center", "tjunc_e"], {"auto_base": True}),
    ("cross_straight", ["cross_w", "cross_center", "cross_e"], {"auto_base": True}),
    ("cross_left", ["cross_n", "cross_center", "cross_e"], {"auto_base": True}),
    ("y_fork_right", ["y_stem", "y_center", "y_sw"], {"auto_base": True}),
    ("y_fork_left", ["y_stem", "y_center", "y_se"], {"auto_base": True}),
    # one-way through a 4-way: no centreline bound on the arms.
    ("oneway_through", ["ow_w", "ow_center", "ow_e"], {"auto_base": True}),
    # one-way -> two-way: auto-base diagonal + dot bound eases in on exit.
    ("oneway_to_two_way", ["ow_w", "ow_center", "ow_s"], {"auto_base": True}),
    ("s_curve", ["s_p0", "s_p1", "s_p2", "s_p3", "s_p4"], {"auto_base": True}),
    ("hairpin", ["hair_a", "hair_corner", "hair_b"], {"auto_base": True}),
    # roundabout: one-way ring, junction dots at entry/exit.
    ("roundabout_entry",
     ["rb_north_far"] + [f"rb_r{i}" for i in range(0, 9)], {"auto_base": True}),
    ("roundabout_lap_exit",
     ["rb_north_far"] + [f"rb_r{i}" for i in range(0, 49)] + ["rb_east_far"],
     {"auto_base": True}),
    # parking avenue: 19.4 m, 2 driving + 1 parking lane per side; nominal
    # = centre of the outermost driving lane (5.25 m).
    ("parking_avenue", ["pkw_n", "pkw_s"], {"auto_base": True}),
    ("parking_avenue_spawn_left", ["pkw_n", "pkw_s"],
     {"base_offset": 1.75}),
    # merge-right blend: spawn in the left lane (1.75 m), change to the
    # normal lane (5.25 m) between s=100 and s=140.
    ("parking_avenue_merge", ["pkw_n", "pkw_s"],
     {"base_offset": 5.25, "merge_from_m": 1.75,
      "merge_s0": 100.0, "merge_s1": 140.0}),
    # width steps 13 -> 9 -> 7 m: per-station nominal changes diagonally.
    ("widths_change", ["widths_n", "widths_139", "widths_97", "widths_74"], {"auto_base": True}),
    # one-way INTO a two-way junction (the documented swing-right case).
    ("mixin", ["mixin_w", "mixin_center", "mixin_e"], {"auto_base": True}),
    # two-way approach onto a one-way exit: centreline bound eases out.
    ("mixout", ["mixout_e", "mixout_center", "mixout_w"], {"auto_base": True}),
    # 4.16 m sliver approach into a 4-way, near-straight continuation.
    ("sliver_straight", ["sliv_ap", "sliv_junc", "sliv_str"], {"auto_base": True}),
    # full figure-8 loop (incl. the elevated crossing branch): start at
    # n24, one complete lap back to n24.
    ("fig8_loop",
     [f"fig8_n{(24 + i) % 48}" for i in range(49)], {"auto_base": True}),
]


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "..", "driving-game",
                     "data", "raceline_reference", "basic.json"))

    network = build_basic_test_map()
    cases = []
    for name, nodes, kw in CASES:
        raw = [network.nodes[n] for n in nodes]
        rounded = _round_polyline_corners(
            raw, CORNER_RADIUS_M * PPPM,
            arc_steps=CORNER_ARC_STEPS, fit_edges=True)
        seg_idx = route_seg_indices(network, nodes)

        P, N, offsets, cum = raceline.solve_line(
            network, rounded, seg_idx, **kw)

        # Intermediates (same calls _solve_line_impl makes) for debugging.
        props = raceline._station_segments(network, seg_idx, P)
        _, K = raceline._normals_and_curvature(P, raceline.SAMPLE_M)
        junction = raceline._junction_node_per_station(network, P)
        lo, hi = raceline.legal_corridor(
            network, P, N, props, junction)
        if kw.get("auto_base", False):
            base_prof = raceline._auto_base_profile(props, cum, lo=lo, hi=hi)
        elif "base_offset" in kw:
            base_prof = [kw["base_offset"]] * len(cum)
        else:
            base_prof = [min(config.LANE_OFFSET_DEFAULT_M,
                             config.kerb_offset_m(
                                 raceline._min_width(network, seg_idx)))] \
                * len(cum)

        cases.append({
            "name": name,
            "nodes": nodes,
            "seg_idx": seg_idx,
            "kwargs": kw,
            "rounded": [list(p) for p in rounded],
            "P": [list(p) for p in P],
            "N": [list(v) for v in N],
            "offsets": list(offsets),
            "cum": list(cum),
            # diagnostics (not part of the 1e-9 gate):
            "K": list(K),
            "lo": list(lo),
            "hi": list(hi),
            "base_prof": list(base_prof),
            "props": [[int(o), w, l, p] for o, w, l, p in props],
            "junction": [None if q is None else list(q) for q in junction],
        })

    doc = {
        "map": "basic",
        "pppm": PPPM,
        "corner_radius_m": CORNER_RADIUS_M,
        "corner_arc_steps": CORNER_ARC_STEPS,
        # network sanity (the C# side builds its own map and checks these):
        "node_count": len(network.nodes),
        "segment_count": len(network.segments),
        "nodes": {nid: list(xy) for nid, xy in network.nodes.items()},
        "node_degree": dict(network.node_degree),
        # geometry the C# solver must run on IDENTICAL polygons for the
        # strict gate (raw paved + eroded safe):
        "paved": network.get_paved_polygon().__geo_interface__,
        "safe": network._raceline_safe.__geo_interface__,
        "cases": cases,
    }

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(doc, f, indent=1)
    total_st = sum(len(c["P"]) for c in cases)
    print(f"wrote {out_path}: {len(cases)} cases, {total_st} stations, "
          f"{len(network.nodes)} nodes / {len(network.segments)} segments")


if __name__ == "__main__":
    main()
