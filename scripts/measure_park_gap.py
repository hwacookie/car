"""Direct measurement: gap between the (live or given) car body and the
paved edge, in world coordinates - no character-level reasoning.

Usage: scripts/measure_park_gap.py [x y heading_deg]   (world pixels)
Without args: reads the live game state via REST (needs a running game).
"""
import json
import math
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from src.test_maps import build_test_map
from src.obstacles import player_body_corners
from src.car import Car


def main() -> None:
    if len(sys.argv) >= 4:
        x, y, h = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        seg_idx = None
    else:
        with urllib.request.urlopen("http://localhost:5000/state") as r:
            s = json.load(r)
        x, y, h = s["x"], s["y"], s["heading"]
        seg_idx = s.get("segment")

    net = build_test_map("basic")
    car = Car(x, y, h, seg_idx if seg_idx is not None else 0)

    # Nearest segment to the car (if unknown).
    if seg_idx is None:
        best, bd = 0, float("inf")
        for i, sg in enumerate(net.segments):
            d = math.dist((car.x, car.y), ((sg.x1 + sg.x2) / 2, (sg.y1 + sg.y2) / 2))
            if d < bd:
                best, bd = i, d
        seg_idx = best

    seg = net.segments[seg_idx]
    dxs, dys = seg.x2 - seg.x1, seg.y2 - seg.y1
    L = math.hypot(dxs, dys)
    nx, ny = dys / L, -dxs / L          # right normal (deg frame: 0=N)
    half_w = seg.width / 2.0

    corners = player_body_corners(car)
    print(f"car ({x:.1f}, {y:.1f}) heading {h:.1f} deg, segment {seg_idx} "
          f"(width {seg.width:g} m)")
    gmin = float("inf")
    for (cx, cy) in corners:
        d_lat = ((cx - seg.x1) * nx + (cy - seg.y1) * ny) / 2.0   # meters
        gap_r = half_w - d_lat          # to right curb
        gap_l = half_w + d_lat          # to left curb
        g, side = (gap_r, "R") if gap_r <= gap_l else (gap_l, "L")
        gmin = min(gmin, g)
        print(f"  corner ({cx:.0f},{cy:.0f}): {g:.3f} m to "
              f"{'right' if side == 'R' else 'left'} curb")
    # rear-axle lateral offset from centreline (meters):
    d_lat_axle = ((car.x - seg.x1) * nx + (car.y - seg.y1) * ny) / 2.0
    print(f"min body-edge gap to curb (segment frame): {gmin:.3f} m")
    print(f"rear-axle offset from centreline: {d_lat_axle:+.3f} m "
          f"(right +)")
    print(f"kerb at +/-{half_w:.2f} m; body half-width 0.9 m")

    # Exact: distance of each body corner to the paved polygon boundary.
    from shapely.geometry import Point
    poly = net.get_paved_polygon()
    bnd = poly.boundary if hasattr(poly, "boundary") else poly
    print("distance to paved-polygon boundary (Shapely):")
    for i, (cx, cy) in enumerate(corners):
        d_m = Point(cx, cy).distance(bnd) / 2.0
        print(f"  corner {i} ({cx:.0f},{cy:.0f}): {d_m:.3f} m")


if __name__ == "__main__":
    main()
