#!/usr/bin/env python3
"""Standalone debug for any named start point + turn direction.

Replays the exact game flow (teleport to start point -> bicycle nav update
loop) without the pygame/Flask stack, and prints the nav's internal state
every ~0.5 s so we can see WHY the car behaves the way it does.

Usage: .venv/bin/python scripts/debug_turn.py <start_point> <direction> [seconds]
   e.g. .venv/bin/python scripts/debug_turn.py corner_right_entry right 12
"""
import sys
import math

import pygame

from src.test_maps import build_basic_test_map
from src.car import Car
from src.driver import BicycleDriver
from src.bicycle_nav import BicycleNav

START = sys.argv[1] if len(sys.argv) > 1 else "corner_right_entry"
DIRECTION = sys.argv[2] if len(sys.argv) > 2 else "right"
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
DT = 1 / 60
PPPM = 2.0


def main():
    network = build_basic_test_map()
    x, y, h, seg, fwd = network.get_start_point(START)
    car = Car(x, y, h, seg, driver=BicycleDriver())
    car.progress = 0.0 if fwd else 1.0
    car.forward = fwd
    car.speed = 0.0
    car.bicycle_nav = BicycleNav(car, network)
    car.bicycle_nav.reset()

    if DIRECTION == "left":
        car.driver.pending_turn = "left"
        car.driver.blinker_left = True
    elif DIRECTION == "right":
        car.driver.pending_turn = "right"
        car.driver.blinker_right = True

    nav = car.bicycle_nav
    print(f"start={START} dir={DIRECTION} spawn=({x:.1f},{y:.1f}) hdg={h:.1f} seg={seg} fwd={fwd}")

    # Dump the initial reference line (before any motion).
    ref = nav._ref
    if ref:
        total = ref.total
        print(f"  ref_total={total:.2f} m")
        print(f"  {'s':>6} {'x':>8} {'y':>8}  hdg_along_ref(deg)")
        for ss in [0, 2, 4, 6, 8, 10, 14, 18, 24, 30, 40, 50]:
            if ss > total:
                break
            px, py = ref.point_at(ss)
            px2, py2 = ref.point_at(min(ss + 0.5, total))
            dh = math.degrees(math.atan2(px2 - px, py2 - py))
            print(f"  {ss:6.1f} {px:8.1f} {py:8.1f}  {dh:8.1f}")

    print(f"{'t':>5} {'s':>6} {'v':>6} {'v_tgt':>6} {'delta':>7} {'scale':>6} "
          f"{'hdg':>7} {'seg':>4}  on_road")

    # Mirror the game loop: driver computes controls from (empty) keys, the
    # "test" merges accelerate=True and - like the real test suite - flicks
    # the turn blinker once we are within 50 m of the junction.
    keys = {k: False for k in (pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT,
                               pygame.K_RIGHT, pygame.K_a, pygame.K_d,
                               pygame.K_w, pygame.K_s)}
    blinker_sent = DIRECTION == "straight"

    for frame in range(int(SECONDS * 60)):
        control = car.driver.get_control(car, network, DT, keys)
        control["accelerate"] = True  # the test holds the gas
        if not blinker_sent:
            seg = network.segments[car.seg_idx]
            dist_junc = ((1.0 - car.progress) if car.forward else car.progress) * seg.length
            if dist_junc <= 50.0:
                blinker_sent = True
                if DIRECTION == "left":
                    car.driver.pending_turn = "left"
                    car.driver.blinker_left = True
                    car.driver.blinker_right = False
                elif DIRECTION == "right":
                    car.driver.pending_turn = "right"
                    car.driver.blinker_right = True
                    car.driver.blinker_left = False
        car.update(DT, network, control)

        if frame % 30 == 0:
            t = frame * DT
            ref = nav._ref
            total = ref.total if ref else -1
            s = nav._s
            v = car.speed
            v_tgt = nav._target_speed(s) if ref else -1
            if ref:
                la = 4.0 + 0.5 * car.speed
                tx, ty = ref.point_at(s + la)
                dx = (tx - car.x) / PPPM
                dy = (ty - car.y) / PPPM
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
                  f"{'OK ' if on_road else 'OFF!'}")


if __name__ == "__main__":
    main()
