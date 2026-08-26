"""Empirical kerb-clearance sweep (the 'reinforcement learning' idea).

Question: how close to the kerb can the car park WITHOUT the off-road
detection tripping? Instead of deriving it from formulas, we run the real
manoeuvre headlessly for a dozen-plus candidate values of
config.PARK_KERB_CLEARANCE_M and let the game's own four-corner off-road
check (Car.is_on_road -> RoadNetwork.is_car_on_road, the same detection
the live game and the e2e suite use) be the oracle.

Scenario (per user: no corner needed): spawn on the long straight of the
basic map, destination 30 m ahead, right blinker armed from t=0 - the nav
pulls over and parks (forward or reverse-in, whatever it decides).

Every frame is checked with the off-road detection; the run reports the
first violation (if any), the parking style, the final body-edge gap to
the kerb line, and the parallel error at rest.

Run: .venv/bin/python scripts/sweep_park_clearance.py [start_point]
"""
import math
import sys

sys.path.insert(0, ".")
sys.path.insert(0, "scripts")

from src import config
from src.test_maps import build_test_map
from src.obstacles import player_body_corners
from trace_park import FakeKeys, spawn_at, road_heading_deg

DT = 1.0 / 60.0
PPPM = config.PIXELS_PER_METER
DEST_M = 30.0          # destination ahead on the straight (m)
MAX_SIM_S = 120.0


def run_once(network, clearance: float, start: str) -> dict:
    """One full pull-over at the given PARK_KERB_CLEARANCE_M."""
    config.PARK_KERB_CLEARANCE_M = clearance
    car = spawn_at(network, start)
    car.driver.blinker_right = True
    keys = FakeKeys()
    rad = math.radians(car.heading)
    dest_x = car.x + math.sin(rad) * DEST_M * PPPM
    dest_y = car.y + math.cos(rad) * DEST_M * PPPM

    t = 0.0
    dest_set = False
    first_off = None
    parked = False
    for _ in range(int(MAX_SIM_S / DT)):
        t += DT
        control = car.driver.get_control(car, network, DT, keys)
        control["accelerate"] = True
        car.update(DT, network, control)
        nav = car.bicycle_nav
        if not dest_set and nav is not None and nav._ref is not None:
            nav.set_destination(dest_x, dest_y)
            dest_set = True
        if first_off is None and not car.is_on_road(network):
            first_off = (t, car.x / PPPM, car.y / PPPM)
        if getattr(nav, "_parked", False) and car.speed == 0.0:
            parked = True
            break

    # --- final metrics -------------------------------------------------
    seg = network.segments[car.seg_idx]
    rh = road_heading_deg(network, car)
    par_err = ((car.heading - rh + 180.0) % 360.0) - 180.0
    dxs, dys = seg.x2 - seg.x1, seg.y2 - seg.y1
    L = math.hypot(dxs, dys)
    nx, ny = dys / L, -dxs / L
    half_w = seg.width / 2.0
    gap = None
    for (cx, cy) in player_body_corners(car):
        o = ((cx - seg.x1) * nx + (cy - seg.y1) * ny) / PPPM
        g = min(half_w - o, half_w + o)
        gap = g if gap is None else min(gap, g)
    nav = car.bicycle_nav
    return {
        "c": clearance,
        "style": getattr(nav, "_park_style", "?"),
        "tuck": getattr(nav, "_park_tuck", 0.0),
        "offroad": first_off,
        "gap": gap,
        "par_err": par_err,
        "parked": parked,
        "t": t,
    }


def main():
    start = sys.argv[1] if len(sys.argv) > 1 else "straight"
    network = build_test_map("basic")

    values = [round(0.30 - 0.02 * i, 2) for i in range(16)]   # 0.30..0.00
    print(f"# sweep PARK_KERB_CLEARANCE_M over {values[0]:.2f}..{values[-1]:.2f}"
          f" (step 0.02), start={start}, destination {DEST_M:.0f} m ahead")
    print(f"{'clear':>6} {'style':>8} {'tuck':>5} {'offroad?':>9} "
          f"{'gap(m)':>7} {'parErr':>7}  note")
    results = []
    for c in values:
        r = run_once(network, c, start)
        results.append(r)
        off = r["offroad"]
        gap = f"{r['gap']:.2f}" if r["gap"] is not None else "  -  "
        note = ""
        if off:
            note = f"TRIPPED t={off[0]:.1f}s at ({off[1]:.0f},{off[2]:.0f})px"
        elif not r["parked"]:
            note = "NOT PARKED (timeout)"
        print(f"{r['c']:6.2f} {r['style']:>8} {r['tuck']:5.2f} "
              f"{'YES' if off else 'no':>9} {gap:>7} {r['par_err']:+7.2f}  {note}")

    ok = [r for r in results if r["offroad"] is None and r["parked"]]
    if ok:
        best = min(ok, key=lambda r: r["gap"])
        print(f"\n# closest clean park: clearance={best['c']:.2f} "
              f"style={best['style']} gap={best['gap']:.2f} m "
              f"parErr={best['par_err']:+.2f} deg")


if __name__ == "__main__":
    main()
