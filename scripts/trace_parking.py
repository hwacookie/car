"""Reproduce + log: corner_right scenario (e2e test 1), high-rate trace.

Replicates tests/test_turning.py's monitor_turn exactly for the
'corner_right_entry' / right / 80 km/h scenario against the LIVE game,
and records /state at ~30 Hz so the whole maneuver - especially the
tail (parking) - can be analysed: speed profile (stutter?), brake onset,
blinker timing, lateral drift to the kerb, speed during the swing, stop
heading vs segment heading, blinker-off.

The game must already be running with the car AT this start point:
    python -m src.main --map basic --api --start corner_right_entry

Usage: .venv/bin/python scripts/trace_parking.py [seconds]
"""
import csv
import json
import sys
import time
import urllib.request

API = "http://127.0.0.1:5000"
START_POINT = "corner_right_entry"
TARGET_KMH = 80.0
SIGNAL_DISTANCE_M = 50.0


def post(path, payload):
    req = urllib.request.Request(
        f"{API}{path}", data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.load(r)


def get_state():
    with urllib.request.urlopen(f"{API}/state", timeout=3) as r:
        return json.load(r)


def main():
    total_s = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

    # --- replicate the harness setup exactly ---
    post("/teleport", {"start_point": START_POINT})
    time.sleep(0.5)
    post("/control", {"accelerate": True})          # latched like the harness

    rows = []
    t0 = time.time()
    signaled = False
    sig_t = None
    last = get_state()
    while time.time() - t0 < total_s:
        s = get_state()
        t = time.time() - t0
        # signal right once within 50 m of the junction (harness logic)
        if not signaled and s.get("distance_to_junction") is not None \
                and s["distance_to_junction"] <= SIGNAL_DISTANCE_M:
            post("/control", {"blinker_right": True})
            signaled = True
            sig_t = t
            print(f"[t={t:5.1f}] blinker_right sent at "
                  f"{s['distance_to_junction']:.0f} m before junction, "
                  f"speed {s['speed_kmh']:.0f} km/h")
        rows.append((t, s["speed_kmh"], s["x"], s["y"], s["heading"],
                     s["segment"], s.get("blinker_right", False),
                     s.get("on_road", True),
                     s.get("distance_to_junction")))
        # stop early once fully stopped and settled for 2 s
        if t > 8.0 and s["speed_kmh"] < 0.3 and \
                last["speed_kmh"] < 0.3:
            rows.append((t + 0.05, s["speed_kmh"], s["x"], s["y"],
                         s["heading"], s["segment"],
                         s.get("blinker_right", False),
                         s.get("on_road", True),
                         s.get("distance_to_junction")))
            print(f"[t={t:5.1f}] settled at 0 km/h - stopping trace")
            break
        last = s
        time.sleep(0.033)

    out = "/tmp/trace_corner_right.csv"
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "kmh", "x", "y", "heading", "segment",
                    "blinker_right", "on_road", "dist_junction"])
        w.writerows(rows)
    print(f"saved {len(rows)} rows -> {out}")

    # --- compact tail view: from 10 s (or first time under 45 km/h) ---
    start_i = next((i for i, r in enumerate(rows) if r[1] < 45.0), len(rows))
    start_i = max(0, min(start_i, len(rows) - 1))
    print(f"\n--- tail from t={rows[start_i][0]:.1f}s "
          f"({len(rows) - start_i} samples @ ~30 Hz) ---")
    print(f"{'t':>5} {'kmh':>6} {'x':>6} {'y':>6} {'hdg':>6} {'seg':>4} "
          f"{'bR':>2} {'dJ':>6}")
    for r in rows[start_i:]:
        print(f"{r[0]:5.1f} {r[1]:6.1f} {r[2]:6.0f} {r[3]:6.0f} "
              f"{r[4]:6.1f} {r[5]:4d} {'Y' if r[6] else '.':>2} "
              f"{('' if r[8] is None else format(r[8], '.0f')):>6}")


if __name__ == "__main__":
    main()
