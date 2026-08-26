"""Headless replica of ONE e2e scenario, with a parking trace.

Mirrors tests/test_turning.py: spawn at a named start point, latch the
throttle, signal the turn 50 m before the junction, put the RED end flag
at 50 % of the expected end segment (which the game turns into the nav's
destination), then log the whole approach relative to the END SEGMENT's
centreline so the pull-over can be judged from data:

    t  kmh  seg  phase  d_stop  lat  gap  hdgerr  blinkR

Usage:
  .venv/bin/python scripts/sim_scenario.py corner_left_entry left 8
  .venv/bin/python scripts/sim_scenario.py corner_right_entry right 6
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

DT = 1.0 / 60.0
PPPM = config.PIXELS_PER_METER
SIGNAL_DISTANCE_M = 50.0


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
    advance_m = 0.5 * seg.length          # the harness spawns at the MID point
    rx += math.sin(rad) * advance_m * PPPM
    ry += math.cos(rad) * advance_m * PPPM
    car = Car(rx, ry, rh, seg_idx, BicycleDriver())
    car.progress = 0.5 if fwd else 0.5
    car.forward = fwd
    return car


def main():
    start, direction, end_seg = sys.argv[1], sys.argv[2], int(sys.argv[3])
    network = build_test_map("basic")
    keys = FakeKeys()
    validator = PhysicsValidator(enabled=True)
    lane_guard = LaneGuard(enabled=True)

    seg = network.segments[end_seg]
    L = math.hypot(seg.x2 - seg.x1, seg.y2 - seg.y1)
    nx, ny = (seg.y2 - seg.y1) / L, -(seg.x2 - seg.x1) / L
    seg_heading = math.degrees(math.atan2(seg.x2 - seg.x1,
                                          seg.y2 - seg.y1)) % 360.0

    car = spawn_at(network, start)
    t = 0.0
    signaled = False
    flag_pending = (end_seg, 0.5)
    dest = None
    print(f"end segment {end_seg}: width {seg.width} m, heading "
          f"{seg_heading:.0f}deg, ideal parked lat "
          f"{seg.width / 2 - config.CAR_WIDTH / 2 - config.KERB_CLEARANCE_M:.2f} m")
    print(f"{'t':>6} {'kmh':>6} {'seg':>4} {'phase':>8} {'d_stop':>7} "
          f"{'lat':>6} {'gap':>6} {'hdgerr':>7} {'bl':>3}")

    for frame in range(int(120 * 60)):
        t += DT
        control = car.driver.get_control(car, network, DT, keys)
        control["accelerate"] = True
        nav = car.bicycle_nav
        # signal the turn 50 m before the junction (harness logic)
        if not signaled and direction != "straight" and nav is not None:
            _sg = network.segments[car.seg_idx]
            d_j = ((1.0 - car.progress) if car.forward
                   else car.progress) * _sg.length
            if d_j is not None and d_j <= SIGNAL_DISTANCE_M:
                setattr(car.driver, f"blinker_{direction}", True)
                car.driver.pending_turn = direction
                signaled = True
                print(f"# t={t:.2f}: blinker {direction} at {d_j:.0f} m")
        # resolve the red flag into the nav destination (main.py logic)
        if flag_pending and nav is not None:
            fseg, fprog = flag_pending
            if fseg in getattr(nav, "_route_seg_set", set()):
                pos = _flag_position_on_route(network, nav, fseg, fprog)
                if pos:
                    dest = (pos[0], pos[1])
                    nav.set_destination(pos[0], pos[1])
                    flag_pending = None
                    print(f"# t={t:.2f}: destination set at "
                          f"({pos[0]:.0f},{pos[1]:.0f})")
        car.update(DT, network, control)
        if lane_guard.check(car, DT, network):
            print(f"# WRONG-SIDE t={t:.2f} phase={phase if 'phase' in dir() else '?'} "
                  f"seg={car.seg_idx} v={car.speed*3.6:.1f}")
        n0=len(validator.violations)
        validator.check(car, DT, network)
        if len(validator.violations)>n0:
            print(f"# OFFROAD t={t:.2f} v={car.speed*3.6:.1f} seg={car.seg_idx} "
                  f"x={car.x:.1f} y={car.y:.1f} h={car.heading:.1f} phase={getattr(nav,'park_phase','?')}")

        phase = getattr(nav, "park_phase", "none") if nav else "none"
        # worst lateral reach of any BODY CORNER vs the end segment kerb
        hh = math.radians(car.heading)
        fx_, fy_ = math.sin(hh), math.cos(hh)
        rx_, ry_ = math.cos(hh), -math.sin(hh)
        f_ov = 2.7 + config.CAR_LENGTH / 2.0 - config.FRONT_AXLE_OFFSET_M
        r_ov = config.CAR_LENGTH / 2.0 - config.REAR_AXLE_OFFSET_M
        worst_reach = -9e9
        for lf in (f_ov, -r_ov):
            for lr in (config.CAR_WIDTH / 2.0, -config.CAR_WIDTH / 2.0):
                cx_ = car.x + fx_ * lf * PPPM + rx_ * lr * PPPM
                cy_ = car.y + fy_ * lf * PPPM + ry_ * lr * PPPM
                o_ = ((cx_ - seg.x1) * nx + (cy_ - seg.y1) * ny) / PPPM
                if (rx_ * nx + ry_ * ny) < 0:
                    o_ = -o_
                worst_reach = max(worst_reach, o_)
        if phase in ("swerve", "final", "reverse", "stopped") and car.seg_idx == end_seg:
            main.min_gap = min(getattr(main, "min_gap", 9e9),
                               seg.width / 2.0 - worst_reach)
        d_stop = nav.distance_to_destination() if nav else None
        off = ((car.x - seg.x1) * nx + (car.y - seg.y1) * ny) / PPPM
        h = math.radians(car.heading)
        if (math.cos(h) * nx - math.sin(h) * ny) < 0:
            off = -off
        hdg_err = (car.heading - seg_heading + 180.0) % 360.0 - 180.0
        if abs(hdg_err) > 90.0:
            hdg_err = (hdg_err + 180.0) % 360.0 - 180.0
        gap = seg.width / 2.0 - off - config.CAR_WIDTH / 2.0
        if frame % 12 == 0 and 8.0 < t < 13.0:
            try:
                vprof = nav._target_speed(nav._s) * 3.6
            except Exception:
                vprof = float('nan')
            _sg = network.segments[car.seg_idx]
            dj = ((1.0 - car.progress) if car.forward else car.progress) * _sg.length
            print(f"@ t={t:5.2f} v={car.speed*3.6:5.1f} vprof={vprof:5.1f} "
                  f"dj={dj:6.1f} seg={car.seg_idx} phase={phase} "
                  f"s={nav._s:6.1f}/{nav._ref.total:6.1f} turn={car.driver.pending_turn}")
        if car.seg_idx == end_seg and 3280 < car.x < 3300:
            print(f"{t:6.2f} {car.speed*3.6:6.1f} {car.seg_idx:4d} "
                  f"{phase:>8} "
                  f"{(d_stop if d_stop is not None else float('nan')):7.2f} "
                  f"{off:6.2f} {gap:6.2f} {hdg_err:7.2f} "
                  f"x={car.x:.1f} y={car.y:.1f} h={car.heading:.1f} "
                  f"{'Y' if getattr(car.driver, 'blinker_right', False) else '.':>3}")
        if getattr(nav, "_parked", False) and car.speed == 0.0:
            break
        if (phase == "stopped" and car.speed <= 0.0
                and getattr(nav, "_park_style", "forward") != "reverse"):
            break

    print(f"\nPARKED at ({car.x:.1f},{car.y:.1f}) seg={car.seg_idx} "
          f"lat={off:.2f} m gap={gap:.2f} m hdg_err={hdg_err:.2f} deg "
          f"on_road={car.is_on_road(network)}")
    if dest:
        fo = nav.FRONT_OVERHANG_M
        bx = car.x + math.sin(h) * fo * PPPM
        by = car.y + math.cos(h) * fo * PPPM
        fwd = ((bx - dest[0]) * math.sin(h) + (by - dest[1]) * math.cos(h)) / PPPM
        print(f"front bumper vs flag: longitudinal {fwd:+.2f} m")
    print(f"min corner gap to kerb during pull-over: "
          f"{getattr(main, 'min_gap', float('nan')):.2f} m")
    print(f"validator violations: {len(validator.violations)}")
    for v in validator.violations[:5]:
        print("   -", v)
    print("lane guard:", lane_guard.stats())


if __name__ == "__main__":
    main()
