"""Headless replica of the WHOLE deterministic e2e suite (tests/test_turning.py),
run in-process against Car + BicycleDriver + BicycleNav - no game window, no
REST, no display. For fast iteration BEFORE burning a live-game run.

Replicates, per scenario:
  - spawn at the named start point (mid-segment, like the harness)
  - red flag at 50% of the expected end segment -> nav destination
  - blinker armed 50 m before the junction (skip for 'straight')
  - accelerate held down throughout
  - PhysicsValidator + LaneGuard running every frame

Pass criteria mirror docs/TESTING.md §3:
  1. stops at the flag (within STOP_AT_FLAG_TOLERANCE_M, or crosses at
     FLAG_CRAWL_KMH and then stops) - never drives past it moving
  2. ends on exactly the expected segment
  3. zero off-road validator violations (wrong-side is reported only)
  4. no >30 deg/frame heading snap
  5. no teleport (distance <= speed*dt + margin)
  6. arrives inside the scenario's monitor window

Usage: .venv/bin/python scripts/sim_suite.py [name_substring]
"""
import math
import sys

sys.path.insert(0, ".")

from src import config
from src.test_maps import build_test_map
from src.car import Car
from src.driver import BicycleDriver
from src.physics_validator import PhysicsValidator
from src.lane_guard import LaneGuard
from src.main import _flag_position_on_route
from tests.test_turning import DETERMINISTIC_TESTS

DT = 1.0 / 60.0
PPPM = config.PIXELS_PER_METER
SIGNAL_DISTANCE_M = 50.0
STOP_AT_FLAG_TOLERANCE_M = 8.0
FLAG_CRAWL_KMH = 5.0


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
    advance_m = 0.5 * seg.length
    rx += math.sin(rad) * advance_m * PPPM
    ry += math.cos(rad) * advance_m * PPPM
    car = Car(rx, ry, rh, seg_idx, BicycleDriver())
    car.progress = 0.5
    car.forward = fwd
    return car


def run_one(start, direction, target_kmh, end_seg, duration_s, desc):
    network = build_test_map("basic")
    keys = FakeKeys()
    validator = PhysicsValidator(enabled=True)
    lane_guard = LaneGuard(enabled=True)
    car = spawn_at(network, start)

    t = 0.0
    signaled = (direction == "straight")
    flag_pending = (end_seg, 0.5)
    dest = None
    last_x, last_y, last_h = car.x, car.y, car.heading
    max_heading_step = 0.0
    passed_flag_moving = False
    stop_ptr = None
    max_frames = int((duration_s + 5.0) * 60)   # small slack over the window

    for frame in range(max_frames):
        t += DT
        control = car.driver.get_control(car, network, DT, keys)
        control["accelerate"] = True
        nav = car.bicycle_nav
        if not signaled and nav is not None:
            seg_now = network.segments[car.seg_idx]
            d_j = ((1.0 - car.progress) if car.forward
                   else car.progress) * seg_now.length
            if d_j <= SIGNAL_DISTANCE_M:
                setattr(car.driver, f"blinker_{direction}", True)
                car.driver.pending_turn = direction
                signaled = True
        if flag_pending and nav is not None:
            fseg, fprog = flag_pending
            if fseg in getattr(nav, "_route_seg_set", set()):
                pos = _flag_position_on_route(network, nav, fseg, fprog)
                if pos:
                    dest = pos
                    nav.set_destination(pos[0], pos[1])
                    flag_pending = None

        car.update(DT, network, control)
        lane_guard.check(car, DT, network)
        validator.check(car, DT, network)

        dh = abs((car.heading - last_h + 180.0) % 360.0 - 180.0)
        max_heading_step = max(max_heading_step, dh)
        dist_px = math.hypot(car.x - last_x, car.y - last_y)
        max_dist_px = (abs(car.speed) * DT + 0.3) * PPPM
        teleport = dist_px > max_dist_px
        last_x, last_y, last_h = car.x, car.y, car.heading

        if dest is not None and car.seg_idx == end_seg and not passed_flag_moving:
            fx, fy = dest[0], dest[1]
            past = math.hypot(car.x - fx, car.y - fy) / PPPM
            if past < 1.0 and car.speed * 3.6 > FLAG_CRAWL_KMH:
                passed_flag_moving = True

        if car.seg_idx == end_seg and abs(car.speed) < 0.05 and stop_ptr is None:
            stop_ptr = t

        if teleport:
            return dict(ok=False, reason=f"teleport/jump ({dist_px/PPPM:.2f} m "
                        f"in one frame vs {max_dist_px/PPPM:.2f} m budget)",
                        t=t)
        if max_heading_step > 30.0:
            return dict(ok=False, reason=f"heading snap {max_heading_step:.0f} "
                        "deg/frame", t=t)

        if stop_ptr is not None and t - stop_ptr > 1.0:
            break

    off_road = len(validator.violations)
    wrong_seg = car.seg_idx != end_seg
    reasons = []
    if passed_flag_moving:
        reasons.append("drove past the end flag while moving")
    if wrong_seg:
        reasons.append(f"wrong end segment ({car.seg_idx}, expected {end_seg})")
    if off_road:
        reasons.append(f"{off_road} off-road violation(s)")
    if stop_ptr is None:
        reasons.append(f"never stopped on segment {end_seg} within "
                       f"{duration_s:.0f}s window")
    lg = lane_guard.stats()
    return dict(ok=not reasons, reason="; ".join(reasons) or None, t=t,
                off_road=off_road, wrong_side_s=lg['wrong_side_seconds'],
                final_seg=car.seg_idx, stop_t=stop_ptr)


def main():
    filt = sys.argv[1] if len(sys.argv) > 1 else None
    n_pass = n_fail = 0
    for i, (start, direction, target_kmh, end_seg, duration_s, desc) in \
            enumerate(DETERMINISTIC_TESTS, 1):
        if filt and filt not in start and filt not in desc:
            continue
        r = run_one(start, direction, target_kmh, end_seg, duration_s, desc)
        mark = "PASS" if r["ok"] else "FAIL"
        if r["ok"]:
            n_pass += 1
        else:
            n_fail += 1
        print(f"[{mark}] {i:2d}. {start:24s} {direction:8s} -> seg {end_seg:3d}  "
              f"({desc})")
        if not r["ok"]:
            print(f"         reason: {r['reason']}  (t={r['t']:.1f}s)")
        elif r.get("wrong_side_s", 0) > 0:
            print(f"         (note: {r['wrong_side_s']:.2f}s wrong-side, "
                  "reported not failed)")
    print(f"\n{n_pass} passed, {n_fail} failed "
          f"(of {n_pass + n_fail} run{'s' if filt is None else ' matching filter'})")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
