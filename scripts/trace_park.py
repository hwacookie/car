"""High-resolution trace of the FORWARD PARKING tail (repro for: 'steers too
much left at the end, not parallel, could get closer to the edge').

Replicates e2e test 1 headlessly: spawn at 'corner_right_entry' of the
synthetic basic map, right blinker armed, throttle latched; the nav takes
the car through the corner and parks forward at the destination flag.
Logs EVERY frame (60 Hz) once within TRACE_FROM_M of the stop point:
speed, phase, d_stop, car heading vs road heading (parallel error),
reference-line heading + its error, cross-track error e_right, and the
commanded steering angle. At rest it reports the final parallel error and
the per-corner clearance to the pavement edges.

Run: .venv/bin/python scripts/trace_park.py [start_point] [end_segment]

The optional end_segment replicates the e2e suite's RED end flag at 50%
of that segment (the car's destination): without it the route has no
destination and the nav parks FORWARD; with it, the reverse-in style
decision can engage exactly like in the live game.
"""
import math
import sys

sys.path.insert(0, ".")

from src import config
from src.test_maps import build_test_map
from src.car import Car
from src.driver import BicycleDriver
from src.obstacles import player_body_corners

DT = 1.0 / 60.0
PPPM = config.PIXELS_PER_METER
TRACE_FROM_M = 15.0


class FakeKeys:
    def __getitem__(self, key):
        return False


def spawn_at(network, name):
    rx, ry, rh, seg_idx, fwd = network.get_start_point(name)
    seg = network.segments[seg_idx]
    rad = math.radians(rh)
    offset_m = config.kerb_offset_m(seg.width)
    rx += math.cos(rad) * offset_m * PPPM
    ry -= math.sin(rad) * offset_m * PPPM
    advance_m = config.SPAWN_PROGRESS * seg.length
    rx += math.sin(rad) * advance_m * PPPM
    ry += math.cos(rad) * advance_m * PPPM
    car = Car(rx, ry, rh, seg_idx, BicycleDriver())
    car.progress = (config.SPAWN_PROGRESS if fwd else 1.0 - config.SPAWN_PROGRESS)
    car.forward = fwd
    return car


def road_heading_deg(network, car):
    seg = network.segments[car.seg_idx]
    dxs, dys = seg.x2 - seg.x1, seg.y2 - seg.y1
    return math.degrees(math.atan2(dxs, dys)) % 360.0


def main():
    start = sys.argv[1] if len(sys.argv) > 1 else "corner_right_entry"
    end_seg = int(sys.argv[2]) if len(sys.argv) > 2 else None
    network = build_test_map("basic")
    keys = FakeKeys()
    car = spawn_at(network, start)
    car.driver.blinker_right = True          # e2e test 1 setup

    t = 0.0
    traced = False
    dest_set = False
    print(f"# start: {start}, blinker_right armed, throttle latched")
    for frame in range(int(180 * 60)):
        t += DT
        control = car.driver.get_control(car, network, DT, keys)
        control["accelerate"] = True
        car.update(DT, network, control)
        nav = car.bicycle_nav

        # Replicate the e2e red flag: destination at 50% of end_seg, set
        # once the route covers that segment (like main.py's resolution).
        if (end_seg is not None and not dest_set and nav is not None
                and end_seg in getattr(nav, "_route_seg_set", set())):
            seg = network.segments[end_seg]
            route = getattr(nav, "_route", None) or []
            for i in range(len(route) - 1):
                a, b = route[i], route[i + 1]
                if a != b and a in (seg.start_node, seg.end_node) \
                        and b in (seg.start_node, seg.end_node):
                    ax, ay = network.nodes[a]
                    bx, by = network.nodes[b]
                    nav.set_destination(ax + (bx - ax) * 0.5,
                                        ay + (by - ay) * 0.5)
                    dest_set = True
                    print(f"# destination set: seg {end_seg} @ 50%")
                    break

        d_stop = None
        if nav is not None and nav._ref is not None:
            stop_s = nav._ref.total - nav._stop_margin()
            d_stop = stop_s - nav._park_s()

        in_tail = (d_stop is not None and 0.0 <= d_stop < TRACE_FROM_M)
        if in_tail and not traced:
            traced = True
            print(f"{'t':>7} {'kmh':>6} {'phase':>8} {'dstop':>7} "
                  f"{'hdg':>7} {'roadH':>7} {'parErr':>7} "
                  f"{'lineH':>7} {'lineErr':>8} {'eR':>6} {'steer':>7}")
        if in_tail:
            seg = network.segments[car.seg_idx]
            rh = road_heading_deg(network, car)
            par_err = ((car.heading - rh + 180.0) % 360.0) - 180.0
            line_h = line_err = float("nan")
            e_r = float("nan")
            if nav._ref is not None and nav._s is not None:
                line_h = nav._ref.heading_at(nav._s) % 360.0
                line_err = ((line_h - car.heading + 180.0) % 360.0) - 180.0
                ref_x, ref_y = nav._ref.point_at(nav._s)
                lh = math.radians(line_h)
                rnx, rny = math.cos(lh), -math.sin(lh)
                e_r = ((car.x - ref_x) * rnx + (car.y - ref_y) * rny) / PPPM
            print(f"{t:7.2f} {car.speed*3.6:6.1f} "
                  f"{getattr(nav, 'park_phase', 'none'):>8} {d_stop:7.2f} "
                  f"{car.heading % 360:7.2f} {rh:7.2f} {par_err:+7.2f} "
                  f"{line_h:7.2f} {line_err:+8.2f} {e_r:+6.2f} "
                  f"{math.degrees(car.steer_angle):+7.1f}")

        if getattr(nav, "_parked", False) and car.speed == 0.0:
            break
        if (getattr(nav, "park_phase", "none") == "stopped"
                and car.speed <= 0.0
                and getattr(nav, "_park_style", "forward") != "reverse"):
            break

    # --- final report -------------------------------------------------
    nav = car.bicycle_nav
    seg = network.segments[car.seg_idx]
    rh = road_heading_deg(network, car)
    par_err = ((car.heading - rh + 180.0) % 360.0) - 180.0
    print(f"\n# FINAL: style={getattr(nav,'_park_style','?')} "
          f"phase={getattr(nav,'park_phase','?')} v={car.speed:.2f} m/s")
    print(f"#   car heading {car.heading % 360:.2f} vs road {rh:.2f} "
          f"-> parallel error {par_err:+.2f} deg")
    dxs, dys = seg.x2 - seg.x1, seg.y2 - seg.y1
    L = math.hypot(dxs, dys)
    nx, ny = dys / L, -dxs / L
    half_w = seg.width / 2.0
    print(f"#   segment {car.seg_idx}: width {seg.width:.1f} m "
          f"(edges at +-{half_w:.2f} m from centreline)")
    for i, (cx, cy) in enumerate(player_body_corners(car)):
        o = ((cx - seg.x1) * nx + (cy - seg.y1) * ny) / PPPM
        print(f"#   corner {i}: lateral offset {o:+.2f} m "
              f"(clearance to nearer edge {min(half_w - o, half_w + o):.2f} m)")
    if nav._ref is not None:
        for dd in (0.1, 2.0, 5.0):
            s = max(0.0, nav._ref.total - dd)
            px, py = nav._ref.point_at(s)
            o = ((px - seg.x1) * nx + (py - seg.y1) * ny) / PPPM
            lh = nav._ref.heading_at(s) % 360.0
            le = ((lh - rh + 180.0) % 360.0) - 180.0
            print(f"#   ref line {dd:4.1f} m before end: offset {o:+.2f} m, "
                  f"heading {lh:.2f} ({le:+.2f} vs road)")


if __name__ == "__main__":
    main()
