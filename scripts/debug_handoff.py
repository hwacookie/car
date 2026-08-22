#!/usr/bin/env python3
"""Debug: log per-frame state around the arc-end hand-off of turn 818->746."""

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
MAX_FRAMES = 60 * 20
LO, HI = 244, 262  # frames of interest


def main():
    pygame.init()
    screen = pygame.display.set_mode((640, 360))

    bb = config.BOUNDING_BOX
    osm_data = fetch_osm_data(bb["north"], bb["south"], bb["west"], bb["east"])
    network = RoadNetwork.from_osm_data(osm_data, bb["north"], bb["south"], bb["west"], bb["east"])

    driver = AIDriver()
    driver.pending_turn = "left"
    driver.blinker_left = True

    seg = network.segments[815]
    car = Car(seg.x1, seg.y1, 0.0, 815, driver)
    car.progress = 0.5
    car.forward = True
    car.speed = 15.0
    car.target_speed = config.CAR_SPEED
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

    print(f"{'frame':>5} {'seg':>4} {'prog':>7} {'speed':>6} {'heading':>8} "
          f"{'x':>9} {'y':>9} {'factor':>7} {'dxy':>8} {'active':>6} {'plan':>14}")
    prev = None
    for frame in range(MAX_FRAMES):
        car.update(DT, network, control)
        try:
            validator.check(car, DT, network)
        except RuntimeError as e:
            print(f"\nCRASH at frame {frame}: {str(e).splitlines()[4] if str(e) else e}")
            break
        dxy = ""
        if prev:
            d = math.hypot(car.x - prev[0], car.y - prev[1]) / config.PIXELS_PER_METER
            dxy = f"{d:.3f}"
        plan_str = f"{car.planned_turn_key}" if car.planned_turn else (
            "none" if car.planned_turn_key is None else str(car.planned_turn_key))
        act = "TURN" if car.active_turn else ""
        if LO <= frame <= HI:
            print(f"{frame:5d} {car.seg_idx:4d} {car.progress:7.4f} "
                  f"{car.speed * 3.6:6.1f} {car.heading:8.1f} "
                  f"{car.x:9.2f} {car.y:9.2f} {car._lane_offset_factor:7.4f} "
                  f"{dxy:>8} {act:>6} {plan_str:>14}")
        prev = (car.x, car.y)
    pygame.quit()


if __name__ == "__main__":
    main()
