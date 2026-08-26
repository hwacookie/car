"""Trace the PARKING TAIL of one e2e scenario against the live game.

Replicates tests/test_turning.py's setup for a single deterministic
scenario (teleport, red end flag at 50% of the expected end segment,
latched throttle, blinker at 50 m before the junction) and logs, at
~20 Hz, where the car is RELATIVE TO THE END SEGMENT'S CENTRELINE:

    t  kmh  seg  lat_off  gap_to_kerb  hdg_err  blinkerR

lat_off is positive to the car's right, so a proper pull-over shows it
growing towards (road_width/2 - car_width/2) in the last ~12 m.

Usage:
  .venv/bin/python scripts/trace_park_scenario.py corner_left_entry left 8
  .venv/bin/python scripts/trace_park_scenario.py corner_right_entry right 6
"""
import json
import math
import sys
import time
import urllib.request

sys.path.insert(0, ".")
from src import config           # noqa: E402

API = "http://127.0.0.1:5000"
PPPM = config.PIXELS_PER_METER
SIGNAL_DISTANCE_M = 50.0


def post(path, payload):
    req = urllib.request.Request(
        f"{API}{path}", data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.load(r)


def get(path):
    with urllib.request.urlopen(f"{API}{path}", timeout=3) as r:
        return json.load(r)


def main():
    start = sys.argv[1]
    direction = sys.argv[2]
    end_seg = int(sys.argv[3])
    total_s = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0

    from src.test_maps import build_test_map
    net = build_test_map("basic")
    seg = net.segments[end_seg]
    x1, y1, x2, y2 = seg.x1, seg.y1, seg.x2, seg.y2
    width = seg.width
    L = math.hypot(x2 - x1, y2 - y1)
    nx, ny = (y2 - y1) / L, -(x2 - x1) / L      # a normal of the segment
    seg_heading = math.degrees(math.atan2(x2 - x1, y2 - y1)) % 360.0

    post("/teleport", {"start_point": start})
    time.sleep(0.5)
    st = get("/state")
    post("/flags", {"green": [st["x"], st["y"], st["heading"]],
                    "red": [end_seg, 0.5]})
    post("/control", {"accelerate": True})

    print(f"end segment {end_seg}: width {width} m, heading {seg_heading:.0f}°"
          f"  (ideal parked lat_off "
          f"{width / 2 - config.CAR_WIDTH / 2 - config.KERB_CLEARANCE_M:.2f} m)")
    print(f"{'t':>5} {'kmh':>6} {'seg':>4} {'lat':>6} {'gap':>6} "
          f"{'hdgerr':>7} {'bl':>3}")

    t0 = time.time()
    signaled = False
    rows = []
    while time.time() - t0 < total_s:
        s = get("/state")
        t = time.time() - t0
        if (not signaled and direction != "straight"
                and s.get("distance_to_junction") is not None
                and s["distance_to_junction"] <= SIGNAL_DISTANCE_M):
            post("/control", {f"blinker_{direction}": True})
            signaled = True
        # lateral offset from the END segment centreline, positive to the
        # car's right
        off = ((s["x"] - x1) * nx + (s["y"] - y1) * ny) / PPPM
        h = math.radians(s["heading"])
        rx, ry = math.cos(h), -math.sin(h)
        if (rx * nx + ry * ny) < 0:
            off = -off
        hdg_err = (s["heading"] - seg_heading + 180.0) % 360.0 - 180.0
        if abs(hdg_err) > 90.0:      # segment traversed backwards
            hdg_err = (hdg_err + 180.0) % 360.0 - 180.0
        gap = width / 2.0 - off - config.CAR_WIDTH / 2.0
        rows.append((t, s["speed_kmh"], s["segment"], off, gap, hdg_err,
                     s.get("blinker_right", False)))
        if s["segment"] == end_seg and s["speed_kmh"] < 0.2 and t > 5:
            break
        time.sleep(0.05)

    for r in rows:
        if True:
            print(f"{r[0]:5.1f} {r[1]:6.1f} {r[2]:4d} {r[3]:6.2f} {r[4]:6.2f} "
                  f"{r[5]:7.2f} {'Y' if r[6] else '.':>3}")
    last = rows[-1]
    print(f"\nPARKED: lat_off {last[3]:.2f} m, gap flank->kerb {last[4]:.2f} m, "
          f"heading error {last[5]:.2f}°, speed {last[1]:.2f} km/h")


if __name__ == "__main__":
    main()
