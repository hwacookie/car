"""Headless repro of the parking maneuver (docs/DRIVING_MANEUVERS.md §1).

Spawns on the 'widths' dead-end street of the synthetic 'basic' map,
drives with the throttle latched (like the e2e harness) and lets the nav's
brake & park plan take over. Logs the whole tail at 10 Hz: speed,
deceleration, park phase, lateral offset from the kerb, heading error and
distance to the stop point, so the animation can be judged from data.

Run: .venv/bin/python scripts/sim_park.py [--dest]
  --dest  set an explicit destination (red flag) instead of the dead end.
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

DT = 1.0 / 60.0
PPPM = config.PIXELS_PER_METER


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


def main():
    use_dest = "--dest" in sys.argv
    network = build_test_map("basic")
    keys = FakeKeys()
    validator = PhysicsValidator(enabled=True)
    lane_guard = LaneGuard(enabled=True)

    start = next((a for a in sys.argv[1:] if not a.startswith("--")), "dead_end_approach")
    print(f"# start point: {start}")
    car = spawn_at(network, start)
    t = 0.0
    prev_v = 0.0
    dest_set = False
    print(f"{'t':>6} {'kmh':>6} {'a':>6} {'phase':>8} {'d_stop':>7} "
          f"{'lat':>6} {'hdgerr':>7} {'blinkR':>6}")
    for frame in range(int(180 * 60)):
        t += DT
        control = car.driver.get_control(car, network, DT, keys)
        control["accelerate"] = True
        car.update(DT, network, control)
        nav = car.bicycle_nav

        if use_dest and not dest_set and nav is not None and nav._ref is not None \
                and t > 3.0:
            # flag 60 m down the line from the car's current position
            sx, sy = nav._ref.point_at(min(nav._ref.total, nav._s + 60.0))
            nav.set_destination(sx, sy)
            dest_set = True
            print(f"# destination set at ({sx:.0f},{sy:.0f}) t={t:.1f}")

        lane_guard.check(car, DT, network)
        n0 = len(validator.violations)
        validator.check(car, DT, network)
        if len(validator.violations) > n0 and not getattr(main, 'seen', False):
            main.seen = True
            print(f"# FIRST VIOLATION t={t:.2f} v={car.speed*3.6:.1f} "
                  f"phase={getattr(nav,'park_phase','?')} "
                  f"d={nav.distance_to_destination()}")

        a = (car.speed - prev_v) / DT
        prev_v = car.speed
        phase = getattr(nav, "park_phase", "none") if nav else "none"
        d_stop = nav.distance_to_destination() if nav else None
        if d_stop is not None and nav._dest is None:
            d_stop -= nav.CAR_LENGTH_M
        lat = hdg_err = float("nan")
        if nav is not None and nav._ref is not None and nav._s is not None:
            rx, ry = nav._ref.point_at(nav._s)
            lat = math.hypot(car.x - rx, car.y - ry) / PPPM
            hdg_err = ((nav._ref.heading_at(nav._s) - car.heading + 180.0)
                       % 360.0 - 180.0)
        if nav is not None and nav._ref is not None and phase in ("swerve","final","decel"):
            seg0 = network.segments[car.seg_idx]
            dxs0, dys0 = seg0.x2 - seg0.x1, seg0.y2 - seg0.y1
            L0 = math.hypot(dxs0, dys0)
            nxx, nyy = dys0 / L0, -dxs0 / L0
            px, py = nav._ref.point_at(max(0.0, nav._ref.total - 3.5))
            oo = ((px - seg0.x1) * nxx + (py - seg0.y1) * nyy) / PPPM
            if frame % 30 == 0:
                print(f"# t={t:.2f} phase={phase} line_off_at_stop={oo:.2f} total={nav._ref.total:.1f}")
        if (frame % 6 == 0 or car.speed * 3.6 < 6.0) \
                and (phase != "none" or car.speed < 1.0):
            print(f"{t:6.2f} {car.speed*3.6:6.1f} {a:6.2f} {phase:>8} "
                  f"{(d_stop if d_stop is not None else float('nan')):7.2f} "
                  f"{lat:6.2f} {hdg_err:7.2f} "
                  f"{'Y' if getattr(car.driver,'blinker_right',False) else '.':>6}")
        if getattr(nav, "_parked", False) and car.speed == 0.0:
            break
        if (phase == "stopped" and car.speed <= 0.0
                and getattr(nav, "_park_style", "forward") != "reverse"):
            break

    if use_dest and nav is not None and nav._dest is not None:
        fo = nav.FRONT_OVERHANG_M
        h = math.radians(car.heading)
        bx = car.x + math.sin(h) * fo * PPPM
        by = car.y + math.cos(h) * fo * PPPM
        d = math.hypot(bx - nav._dest[0], by - nav._dest[1]) / PPPM
        fwd = ((bx - nav._dest[0]) * math.sin(h)
               + (by - nav._dest[1]) * math.cos(h)) / PPPM
        print(f"front bumper to flag: {d:.2f} m (longitudinal {fwd:+.2f} m)")
    if nav is not None and nav._ref is not None:
        seg0 = network.segments[car.seg_idx]
        dxs0, dys0 = seg0.x2 - seg0.x1, seg0.y2 - seg0.y1
        L0 = math.hypot(dxs0, dys0)
        nxx, nyy = dys0 / L0, -dxs0 / L0
        for dd in (0.0, 2.0, 4.0, 8.0, 12.0):
            px, py = nav._ref.point_at(max(0.0, nav._ref.total - dd))
            o = ((px - seg0.x1) * nxx + (py - seg0.y1) * nyy) / PPPM
            print(f"  ref offset {dd:5.1f} m before line end: {o:6.2f} m")
    seg = network.segments[car.seg_idx]
    # lateral offset of the car from the segment centreline (+ = to the
    # car's right) and the gap from its right flank to the pavement edge
    dxs, dys = seg.x2 - seg.x1, seg.y2 - seg.y1
    L = math.hypot(dxs, dys)
    nx_, ny_ = dys / L, -dxs / L          # left-hand normal in screen coords
    off = ((car.x - seg.x1) * nx_ + (car.y - seg.y1) * ny_) / PPPM
    h = math.radians(car.heading)
    right = (math.cos(h), -math.sin(h))
    sign = 1.0 if (right[0] * nx_ + right[1] * ny_) > 0 else -1.0
    off_r = off * sign
    print(f"offset from centreline (right +): {off_r:.2f} m   "
          f"gap flank->kerb: {seg.width / 2.0 - off_r - config.CAR_WIDTH / 2.0:.2f} m")
    print(f"\nfinal: ({car.x:.1f},{car.y:.1f}) h={car.heading:.1f} "
          f"v={car.speed*3.6:.2f} km/h phase={phase} "
          f"d_stop={d_stop if d_stop is None else round(d_stop,2)} "
          f"on_road={car.is_on_road(network)} width={seg.width}")
    print(f"validator violations: {len(validator.violations)}")
    for v in validator.violations[:10]:
        print("  -", v)
    print("lane guard:", lane_guard.stats())
    return 0


if __name__ == "__main__":
    sys.exit(main())
