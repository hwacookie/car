#!/usr/bin/env python3
"""Standalone debug for the sliver_approach stall.

Replays the exact game flow (teleport to start point -> bicycle nav update
loop) without the pygame/Flask stack, and prints the nav's internal state
every ~0.5 s so we can see WHY the car stalls.

Usage: .venv/bin/python scripts/debug_sliver.py [left|right|straight]
"""
import sys

from src.test_maps import build_basic_test_map
from src.car import Car
from src.driver import BicycleDriver
from src.bicycle_nav import BicycleNav

DIRECTION = sys.argv[1] if len(sys.argv) > 1 else "right"
DT = 1 / 60


def main():
    network = build_basic_test_map()
    x, y, h, seg, fwd = network.get_start_point("sliver_approach")
    car = Car(x, y, h, seg, driver=BicycleDriver())
    car.progress = 0.0 if fwd else 1.0
    car.forward = fwd
    car.speed = 0.0
    car._apply_plain_segment_position(network.segments[seg])
    car.bicycle_nav = BicycleNav(car, network)
    car.bicycle_nav.reset()

    # Same as the REST API does: set pending_turn + blinker directly.
    if DIRECTION == "left":
        car.driver.pending_turn = "left"
        car.driver.blinker_left = True
    elif DIRECTION == "right":
        car.driver.pending_turn = "right"
        car.driver.blinker_right = True

    nav = car.bicycle_nav
    print(f"direction={DIRECTION}  spawn=({x:.1f},{y:.1f}) hdg={h:.1f} seg={seg}")
    print(f"{'t':>5} {'s':>6} {'v':>6} {'v_tgt':>6} {'delta':>7} {'scale':>6} "
          f"{'hdg':>7} {'seg':>4}  ref_total")

    for frame in range(60 * 20):
        control = {"accelerate": True}
        if DIRECTION == "left":
            control["blinker_left"] = True
        elif DIRECTION == "right":
            control["blinker_right"] = True
        car.update(DT, network, control)

        if frame % 30 == 0:
            t = frame * DT
            ref = nav._ref
            total = ref.total if ref else -1
            s = nav._s
            v = car.speed
            v_tgt = nav._target_speed(s) if ref else -1
            # Recompute the pursuit delta the same way update() does, for
            # visibility (update() keeps it local).
            import math
            if ref:
                la = 4.0 + 0.5 * car.speed
                tx, ty = ref.point_at(s + la)
                dx = (tx - car.x) / 2.0   # PPPM
                dy = (ty - car.y) / 2.0
                hr = math.radians(car.heading)
                lr = dx * math.cos(hr) - dy * math.sin(hr)
                lf = dx * math.sin(hr) + dy * math.cos(hr)
                if lf < 0.5:
                    lf = 0.5
                delta = math.degrees(math.atan2(lr, lf))
            else:
                delta = float("nan")
            scale = max(0.0, min(1.0, 1.0 - (abs(delta) - 5.0) / 20.0))
            on_road = network.is_on_road(car.x, car.y)
            print(f"{t:5.1f} {s:6.2f} {v:6.2f} {v_tgt:6.2f} {delta:7.1f} "
                  f"{scale:6.2f} {car.heading:7.1f} {car.seg_idx:4d}  "
                  f"{total:6.1f}  {'OK ' if on_road else 'OFF!'}")


if __name__ == "__main__":
    main()
