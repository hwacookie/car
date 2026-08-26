"""Closed-loop dry-run of the U-turn e2e scenario WITHOUT a display.

Drives the REAL game code (Car + BicycleDriver + BicycleNav +
PhysicsValidator + LaneGuard) over the synthetic 'basic' map's WIDTHS
tile: a straight dead-end street in four 50 m sections (13/9/7/4 m).

Scenario (mirrors tests/test_turning.py's flow):
  spawn at 'widths' (north dead end, heading south, in the 13 m section)
  -> pull-out (automatic) -> drive to the middle of the section
  -> request the U-turn (single swing on the 13 m width, spec 5a)
  -> wait for completion -> drive back north -> auto park at the kerb.

Any PhysicsValidator violation raises, exactly like in the live game.
Hazard lights (the car recognising it cannot continue) are a hard FAIL.
Run: .venv/bin/python scripts/sim_uturn.py
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
ENTRY_SPEED_M = 3.5          # ~12.6 km/h entry speed for the maneuver

# Widths tile geometry (meters -> px): section 13 m spans y=300..250 m,
# centreline at x=2100 m.
CENTERLINE_X_PX = 2100.0 * PPPM
SECTION_MID_Y_PX = 275.0 * PPPM        # middle of the 13 m section
ROAD_WIDTH_M = 13.0


class FakeKeys:
    """pygame key state where nothing is pressed."""
    def __getitem__(self, key):
        return False


def spawn_at(network, name):
    """Replicates main._create_car(start_point)."""
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
    network = build_test_map("basic")
    keys = FakeKeys()
    validator = PhysicsValidator(enabled=True)
    lane_guard = LaneGuard(enabled=True)

    spawn = network.get_start_point("widths")
    print(f"spawn: ({spawn[0]:.1f}, {spawn[1]:.1f}) heading {spawn[2]:.0f} deg, "
          f"section middle y={SECTION_MID_Y_PX:.0f} px")

    car = spawn_at(network, "widths")
    t = 0.0
    phase = "drive-to-mid"
    uturn_started = False
    braking_for_uturn = False
    print(f"\n== phase: pull-out + drive to middle of the {ROAD_WIDTH_M:.0f} m section ==")

    max_frames = int(240 * 60)   # 4 minutes of sim time
    for frame in range(max_frames):
        t += DT
        control = car.driver.get_control(car, network, DT, keys)
        # Mirror the e2e harness: it latches accelerate=True via /control.
        # The maneuver needs ~5-10 km/h entry, so brake down before
        # requesting (latched per frame - one-shot control dicts don't persist).
        if phase == "drive-to-mid":
            if car.y <= SECTION_MID_Y_PX + 60.0:
                braking_for_uturn = True
            if braking_for_uturn:
                # Brake down to the entry speed, then HOLD it (throttle) up
                # to the request line - coasting would stop the car short.
                if abs(car.speed) > ENTRY_SPEED_M + 0.2:
                    control["brake"] = True
                    control["accelerate"] = False
                else:
                    control["brake"] = False
                    control["accelerate"] = True
            else:
                control["accelerate"] = True
        elif phase == "return":
            control["accelerate"] = True
        car.update(DT, network, control)

        # The car recognising it cannot continue is a hard failure.
        if getattr(car.driver, "hazard", False):
            print(f"\nFAIL: hazard lights came on - {car.driver.hazard_reason}")
            return 1

        nav = car.bicycle_nav
        uturn_now = nav is not None and getattr(nav, "uturn_active", False)
        in_turn = (nav is not None and nav._s is not None
                   and nav._in_turn_blend_zone(nav._s))
        if not in_turn and not uturn_now:
            lane_guard.check(car, DT, network)
        validator.check(car, DT, network)

        kmh = car.speed * 3.6
        if frame % 60 == 0 or phase != "drive-to-mid" and frame % 30 == 0:
            print(f"  t={t:6.1f}s  ({car.x:7.1f},{car.y:7.1f}) "
                  f"h={car.heading:6.1f} v={kmh:5.1f} km/h seg={car.seg_idx} "
                  f"fwd={car.forward} phase={phase}"
                  + ("  [UTURN]" if uturn_now else ""))

        # --- phase machine (mirrors the e2e test) ---
        if phase == "drive-to-mid":
            if (car.y <= SECTION_MID_Y_PX + 5
                    and abs(car.speed) <= ENTRY_SPEED_M + 0.3):
                print(f"\n== t={t:.1f}s: U-turn requested at "
                      f"({car.x:.1f},{car.y:.1f}) v={kmh:.1f} km/h ==")
                car.driver.uturn_requested = True
                phase = "uturn"
        elif phase == "uturn":
            if not uturn_started and uturn_now:
                print(f"  t={t:.1f}s: U-turn line accepted, maneuver running")
                uturn_started = True
            if uturn_started and not uturn_now:
                # completed: verify the end state of the maneuver
                hdg_err = abs((car.heading + 180.0) % 360 - 180.0)
                print(f"\n== t={t:.1f}s: U-turn COMPLETE at "
                      f"({car.x:.1f},{car.y:.1f}) heading {car.heading:.1f} deg "
                      f"(error vs road axis {hdg_err:.1f} deg), v={kmh:.1f} km/h ==")
                if hdg_err > 30.0:
                    print("FAIL: not facing along the road after U-turn")
                    return 1
                if car.x < CENTERLINE_X_PX:
                    print("FAIL: on the wrong (west) half of the road after U-turn")
                    return 1
                phase = "return"
            elif not uturn_started and frame > 600:
                print("FAIL: U-turn request was rejected (not feasible here)")
                return 1
        elif phase == "return":
            # Auto-parking takes over near the dead end; wait for a full stop.
            if car.speed < 0.1 and t > 5.0:
                hdg_err = abs((car.heading + 180.0) % 360 - 180.0)
                east_kerb_px = (2100.0 + config.kerb_offset_m(ROAD_WIDTH_M)) * PPPM
                print(f"\n== t={t:.1f}s: PARKED at ({car.x:.1f},{car.y:.1f}) "
                      f"heading {car.heading:.1f} deg (err {hdg_err:.1f}), "
                      f"east kerb x={east_kerb_px:.1f} ==")
                ok = True
                if hdg_err > 15.0:
                    print(f"FAIL: not parallel to the road (err {hdg_err:.1f} deg)")
                    ok = False
                if abs(car.x - east_kerb_px) > 4.0:
                    print(f"FAIL: not at the east kerb (x={car.x:.1f}, "
                          f"want {east_kerb_px:.1f})")
                    ok = False
                if not car.is_on_road(network):
                    print("FAIL: parked off-road")
                    ok = False
                stats = lane_guard.stats()
                print(f"lane guard: {stats}")
                print(f"validator violations: {len(validator.violations)}")
                for v in validator.violations[:25]:
                    print("  -", v)
                if len(validator.violations) > 0:
                    ok = False
                print("\nPASS: pull-out -> mid-section U-turn -> return -> park"
                      if ok else "\nFAIL: see above")
                return 0 if ok else 1

    print("FAIL: timed out")
    return 1


if __name__ == "__main__":
    sys.exit(main())
