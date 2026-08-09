#!/usr/bin/env python3
"""Car Game — Entry Point"""

import os
import sys
import pygame

from .config import *
from .osm_loader import fetch_osm_data
from .road_network import RoadNetwork
from .camera import Camera
from .renderer import Renderer
from .car import Car


def main(smoke_test_frames: int = 0):
    """Run the game. If smoke_test_frames > 0, run headless for that many frames."""
    # --- Init ---
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Car Game — Bremen")
    clock = pygame.time.Clock()

    # macOS: bring the window to the foreground (it may open behind the terminal
    # or on another Space/display)
    if sys.platform == "darwin":
        import subprocess
        subprocess.run([
            "osascript", "-e",
            f'tell application "System Events" to set frontmost of '
            f'(first process whose unix id is {os.getpid()}) to true',
        ], capture_output=True)

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
    camera.x, camera.y = car.x, car.y   # snap to car at start
    renderer = Renderer(network, camera)

    # --- Game loop ---
    frame = 0
    running = True
    dt_fixed = 1 / 60
    while running:
        frame += 1
        if smoke_test_frames and frame > smoke_test_frames:
            running = False
            break

        dt = clock.tick(60) / 1000.0 if not smoke_test_frames else dt_fixed

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                camera.handle_zoom(event.y)

        keys = pygame.key.get_pressed()

        # Zoom with +/- keys
        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:
            camera.zoom_in()
        if keys[pygame.K_MINUS]:
            camera.zoom_out()

        # In smoke test, simulate driving inputs
        if smoke_test_frames:
            # accelerate for first half, then steer
            keys = _FakeKeys(accel=True, right=(frame > smoke_test_frames // 2))

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
        renderer.draw(screen, car)
        car.draw(screen, camera)

        pygame.display.flip()

        # Debug: dump framebuffer after a few frames
        if "--dump" in sys.argv and frame == 30:
            pygame.image.save(screen, "/tmp/car_frame.bmp")
            print("Dumped frame to /tmp/car_frame.bmp")

    if smoke_test_frames:
        print(f"Smoke test OK: {frame} frames, car at ({car.x:.0f}, {car.y:.0f}), "
              f"speed={car.speed:.1f} m/s, zoom={camera.zoom:.2f}")
        pygame.quit()
        return

    pygame.quit()
    sys.exit()


class _FakeKeys:
    """Simulates pygame.key.get_pressed() for smoke testing."""
    def __init__(self, accel=False, brake=False, left=False, right=False):
        self._pressed = set()
        if accel:
            self._pressed.add(pygame.K_UP)
        if brake:
            self._pressed.add(pygame.K_DOWN)
        if left:
            self._pressed.add(pygame.K_LEFT)
        if right:
            self._pressed.add(pygame.K_RIGHT)

    def __getitem__(self, key):
        return key in self._pressed


if __name__ == "__main__":
    frames = 0
    if "--smoke" in sys.argv:
        idx = sys.argv.index("--smoke")
        frames = int(sys.argv[idx + 1]) if idx + 1 < len(sys.argv) else 300
    main(smoke_test_frames=frames)
