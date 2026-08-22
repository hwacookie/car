#!/usr/bin/env python3
"""Headless repro for the 815 -> 1008 junction crash (see docs/TURN_REWORK_PLAN.md).

Teleports the car onto seg 815 (the 4.16 m sliver before the 4-way
junction), sets the LEFT blinker (signaling the turn into 1008), and
drives with the accelerator held - the exact scenario that used to make
the game freeze and die with a teleportation RuntimeError.

Run:  .venv/bin/python scripts/repro_crash.py
Exit code 0 = no crash (car survived the junction), 1 = crash/teleport.
"""

import math
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pygame

from src import config
from src.osm_loader import fetch_osm_data
from src.road_network import RoadNetwork
from src.car import Car
from src.driver import AIDriver
from src.physics_validator import PhysicsValidator

DT = 1 / 60
MAX_FRAMES = 60 * 20  # 20 seconds of driving


def main():
    pygame.init()
    screen = pygame.display.set_mode((640, 360))

    bb = config.BOUNDING_BOX
    osm_data = fetch_osm_data(bb["north"], bb["south"], bb["west"], bb["east"])
    network = RoadNetwork.from_osm_data(osm_data, bb["north"], bb["south"], bb["west"], bb["east"])

    driver = AIDriver()
    driver.pending_turn = "left"
    driver.blinker_left = True

    # Teleport onto seg 815, mid-segment, facing the junction (the
    # blinker is thus set ~2 m before the junction - same situation as
    # the reported crash: too late for any physically possible arc).
    seg = network.segments[815]
    car = Car(seg.x1, seg.y1, 0.0, 815, driver)
    car.progress = 0.5
    car.forward = True
    car.speed = 15.0  # ~54 km/h, close to the reported 57 km/h
    car.target_speed = config.CAR_SPEED
    # Face along the segment (world Y points south; heading 0 = +y).
    car.heading = math.degrees(math.atan2(seg.x2 - seg.x1, seg.y2 - seg.y1))
    car._apply_plain_segment_position(seg)

    validator = PhysicsValidator(enabled=True)
    validator.reset_car_state(car)

    control = {
        "accelerate": True,
        "brake": False,
        "steer_left": False,
        "steer_right": False,
        "blinker_left": True,
        "blinker_right": False,
    }

    print(f"\nRepro: car on seg 815 @ {car.speed * 3.6:.0f} km/h, LEFT blinker, "
          f"junction ahead (816 straight / 1007 right / 1008 left)\n")

    for frame in range(MAX_FRAMES):
        car.update(DT, network, control)
        try:
            validator.check(car, DT, network)
        except RuntimeError as e:
            print(f"\n💥 CRASH at frame {frame}: {str(e).splitlines()[4] if str(e) else e}")
            print(f"   (seg {car.seg_idx}, progress {car.progress:.3f}, "
                  f"speed {car.speed * 3.6:.0f} km/h, heading {car.heading:.1f}°)")
            pygame.quit()
            return 1

        if frame % 30 == 0:
            print(f"  frame {frame:4d}: seg {car.seg_idx} prog {car.progress:.3f} "
                  f"speed {car.speed * 3.6:5.1f} km/h heading {car.heading:7.1f}°")

    print(f"\n✅ No crash after {MAX_FRAMES} frames. "
          f"Final: seg {car.seg_idx}, progress {car.progress:.3f}, "
          f"speed {car.speed * 3.6:.0f} km/h, heading {car.heading:.1f}°")
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
