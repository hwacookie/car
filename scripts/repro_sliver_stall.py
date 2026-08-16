#!/usr/bin/env python3
"""Repro for the sliver_approach stall (work-order item 1, TURN_REWORK_PLAN §13.6).

Requires a running game:
    SDL_VIDEODRIVER=dummy python -m src.main --map basic --api --bicycle

Teleports to 'sliver_approach', sets the given blinker + accelerate, and
prints speed/position for ~12 s. A stalled car sits at speed ~0 the whole
time.
"""
import sys
import time

import requests

API = "http://localhost:5000"


def main():
    direction = sys.argv[1] if len(sys.argv) > 1 else "left"
    requests.post(f"{API}/reset")
    requests.post(f"{API}/teleport", json={"start_point": "sliver_approach"})
    time.sleep(0.3)

    ctrl = {"accelerate": True}
    if direction != "straight":
        ctrl[f"blinker_{direction}"] = True
    requests.post(f"{API}/control", json=ctrl)

    t0 = time.time()
    while time.time() - t0 < 12.0:
        st = requests.get(f"{API}/state", timeout=2).json()
        t = time.time() - t0
        print(
            f"t={t:5.1f}s  v={st['speed_kmh']:6.2f} km/h  "
            f"seg={st['segment']:>3}  pos=({st['x']:8.1f},{st['y']:8.1f})  "
            f"hdg={st['heading']:6.1f}°"
        )
        time.sleep(0.5)


if __name__ == "__main__":
    main()
