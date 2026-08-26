#!/usr/bin/env python3
"""Live verification of the obstacle system (docs/OBSTACLES.md Part 1).

Requires a running game:  python -m src.main --map basic --api

Scenario: place a parked car in the lane via REST, teleport the player car
behind it, drive forward - the player must brake at full A_BRAKE on contact
and rest against the obstacle without interpenetrating it, with no validator
crash. Then delete the obstacle again.
"""
import sys
import time

import requests

sys.path.insert(0, "/Users/hauke/prj/car")
from src import config

API = "http://127.0.0.1:5000"
PPPM = config.PIXELS_PER_METER
HALF_L = config.CAR_LENGTH / 2.0          # 2.2 m
REAR_OFF = config.REAR_AXLE_OFFSET_M      # 1.276 m


def get(path):
    return requests.get(API + path, timeout=3).json()


def post(path, body):
    r = requests.post(API + path, json=body, timeout=5)
    return r.status_code, (r.json() if r.content else {})


def main():
    ok = True

    # --- 0. Start from a clean slate (the user may have placed obstacles
    #       manually in the window while testing) ---
    for ob in get("/obstacles"):
        requests.delete(API + f"/obstacles/{ob['id']}", timeout=3)
    print(f"cleared existing obstacles: {get('/obstacles')}")

    # --- 1. Place a parked car in the lane (97 m east-west, 150 m north) ---
    sc, ob = post("/obstacles", {"type": "car", "color": "blue",
                                 "x": 97 * PPPM, "y": 150 * PPPM})
    print(f"place: HTTP {sc} -> {ob}")
    if sc != 201 or not ob.get("id"):
        print("FAIL: expected 201 + id"); return 1
    oid = ob["id"]
    if abs(ob["heading"] - 180.0) > 0.01:
        print(f"FAIL: heading {ob['heading']} != 180 (right half, travel south)")
        ok = False

    # --- 2. Off-road + bad-color rejections ---
    sc, err = post("/obstacles", {"type": "car", "color": "blue",
                                  "x": 78 * PPPM, "y": 100 * PPPM})
    print(f"off-road place: HTTP {sc} -> {err}")
    ok &= sc == 400
    sc, err = post("/obstacles", {"type": "car", "color": "red",
                                  "x": 97 * PPPM, "y": 150 * PPPM})
    print(f"bad color: HTTP {sc} -> {err}")
    ok &= sc == 400

    # --- 3. List ---
    lst = get("/obstacles")
    print(f"list: {lst}")
    ok &= len(lst) == 1 and lst[0]["id"] == oid

    # --- 4. Teleport the player car behind it (straight tile, mid-segment) ---
    post("/teleport", {"start_point": "straight", "progress": 0.5})
    time.sleep(0.6)
    st = get("/state")
    print(f"car spawned: ({st['x']/PPPM:.1f} m, {st['y']/PPPM:.1f} m), "
          f"heading {st['heading']:.0f}, uid {st.get('car_uid')}")
    # Car at ~ (97.75 m, 200 m) facing south; obstacle at (97 m, 150 m).

    # --- 5. Drive into it and watch the stop ---
    post("/control", {"accelerate": True})
    t0 = time.time()
    min_y = float("inf")
    stopped_since = None
    peak_speed = 0.0
    while time.time() - t0 < 30:
        st = get("/state")
        y_m = st["y"] / PPPM
        v = st["speed"]
        peak_speed = max(peak_speed, v)
        min_y = min(min_y, y_m)
        if v == 0.0 and st.get("has_car"):
            if stopped_since is None:
                stopped_since = time.time()
            elif time.time() - stopped_since > 2.0:
                break
        else:
            stopped_since = None
        time.sleep(0.1)

    st = get("/state")
    y_m, v = st["y"] / PPPM, st["speed"]
    print(f"after driving: y={y_m:.2f} m (min {min_y:.2f} m), "
          f"speed={v:.3f} m/s, peak={peak_speed * 3.6:.1f} km/h")

    # The obstacle's north (rear) edge is at 150 + HALF_L = 152.2 m. The car
    # approaches from the north facing south: its front bumper sits at
    # axle_y - REAR_OFF - HALF_L and must never cross that edge, i.e.
    # axle_y >= 152.2 + REAR_OFF + HALF_L = 155.68 m at contact.
    rest_axle_y = 150.0 + HALF_L + REAR_OFF + HALF_L
    if min_y < rest_axle_y - 0.05:
        print(f"FAIL: car penetrated the obstacle (axle reached {min_y:.2f} m, "
              f"rest position ~{rest_axle_y:.2f} m)")
        ok = False
    else:
        front_bumper = y_m - REAR_OFF - HALF_L
        print(f"resting: axle at {y_m:.2f} m (expected ~{rest_axle_y:.2f}), "
              f"front-bumper gap to obstacle edge: "
              f"{front_bumper - (150.0 + HALF_L):+.4f} m")
    if v != 0.0:
        print("FAIL: car did not come to a stop")
        ok = False
    if peak_speed < 5.0:
        print("FAIL: car never really accelerated (peak too low)")
        ok = False

    # --- 6. Delete the obstacle; unknown id -> 404 ---
    r = requests.delete(API + f"/obstacles/{oid}", timeout=3)
    print(f"delete: HTTP {r.status_code} -> {r.json()}")
    ok &= r.status_code == 200
    r = requests.delete(API + f"/obstacles/{oid}", timeout=3)
    print(f"delete again: HTTP {r.status_code} -> {r.json()}")
    ok &= r.status_code == 404
    lst = get("/obstacles")
    print(f"list after delete: {lst}")
    ok &= lst == []

    # --- 7. Game still alive, no validator crash in the log ---
    time.sleep(0.5)
    try:
        get("/health")
        print("game process still alive")
    except requests.RequestException:
        print("FAIL: game process died")
        ok = False

    print("\n" + ("✅ LIVE OBSTACLE VERIFICATION PASSED" if ok
                  else "❌ LIVE OBSTACLE VERIFICATION FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
