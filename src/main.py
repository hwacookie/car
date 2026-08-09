#!/usr/bin/env python3
"""Car Game — Entry Point"""

import sys
import pygame

from .config import *
from .osm_loader import fetch_osm_data
from .road_network import RoadNetwork
from .camera import Camera
from .renderer import Renderer
from .car import Car


def main():
    # --- Init ---
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Car Game — Kleinmachnow")
    clock = pygame.time.Clock()

    # --- Load road data ---
    print("Loading OSM data…")
    bb = BOUNDING_BOX
    osm_data = fetch_osm_data(bb["north"], bb["south"], bb["west"], bb["east"])
    print(f"  {len(osm_data['nodes'])} nodes, {len(osm_data['ways'])} ways")

    # --- Build network ---
    network = RoadNetwork.from_osm_data(osm_data, bb["north"], bb["south"], bb["west"], bb["east"])

    # --- Car on random road ---
    rx, ry, rh = network.random_road_point()
    car = Car(rx, ry, rh)

    # --- Camera + Renderer ---
    camera = Camera(WINDOW_WIDTH, WINDOW_HEIGHT)
    renderer = Renderer(network, camera)

    # --- Game loop ---
    running = True
    while running:
        dt = clock.tick(60) / 1000.0

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                camera.handle_zoom(event.y)

        keys = pygame.key.get_pressed()

        # Zoom with +/- keys
        if keys.get(pygame.K_EQUALS, False) or keys.get(pygame.K_PLUS, False):
            camera.zoom_in()
        if keys.get(pygame.K_MINUS, False):
            camera.zoom_out()

        # Car physics
        car.handle_input(keys, dt)

        # Off-road check
        if not network.is_on_road(car.x, car.y):
            car.speed = 0

        # Map edge check
        bounds = network.bounds
        if car.x < 0 or car.x > bounds[2] or car.y < 0 or car.y > bounds[3]:
            car.speed = 0
            car.x = max(0, min(bounds[2], car.x))
            car.y = max(0, min(bounds[3], car.y))

        # Camera follow
        camera.update(car.x, car.y, network.world_width, network.world_height)

        # Render
        screen.fill(BG_COLOR)
        renderer.draw(screen, car.speed)
        car.draw(screen, camera.zoom)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
