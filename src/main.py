#!/usr/bin/env python3
"""Car Game — Entry Point"""

import math
import os
import sys
import pygame

from .config import *
from .osm_loader import fetch_osm_data
from .road_network import RoadNetwork
from .camera import Camera
from .renderer import Renderer
from .car import Car
from .driver import Driver, KeyboardDriver, AIDriver
from .physics_validator import PhysicsValidator
from .rest_api import GameAPI
from .test_maps import build_test_map, TEST_MAPS


def copy_screenshot_to_clipboard(screen: pygame.Surface) -> None:
    """Save the current frame and copy it to the system clipboard.
    
    Useful for quick bug reports: press ESC in-game to stop the car and
    grab a screenshot you can immediately paste elsewhere.
    """
    import tempfile
    from PIL import Image
    
    # Convert pygame surface -> PIL image (pygame lacks reliable PNG
    # encoding on this platform, see docs/SPEC.md)
    w, h = screen.get_size()
    raw = pygame.image.tostring(screen, 'RGBA')
    img = Image.frombytes('RGBA', (w, h), raw)
    
    path = tempfile.mktemp(suffix='.png')
    img.save(path, 'PNG')
    
    if sys.platform == 'darwin':
        import subprocess
        subprocess.run([
            'osascript', '-e',
            f'set the clipboard to (read (POSIX file "{path}") as «class PNGf»)'
        ], capture_output=True)
        print(f"📷 Screenshot copied to clipboard (saved to {path})")
    else:
        print(f"📷 Screenshot saved to {path} (clipboard copy only supported on macOS)")


def main(smoke_test_frames: int = 0):
    """Run the game. If smoke_test_frames > 0, run headless for that many frames."""
    # --- Init ---
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Car Game — Kleinmachnow")
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
    map_name = None
    if "--map" in sys.argv:
        idx = sys.argv.index("--map")
        if idx + 1 < len(sys.argv):
            map_name = sys.argv[idx + 1]

    if map_name:
        print(f"Loading synthetic test map: '{map_name}'")
        network = build_test_map(map_name)
        print(f"  {len(network.nodes)} nodes, {len(network.segments)} segments")
    else:
        print("Loading OSM data…")
        bb = BOUNDING_BOX
        osm_data = fetch_osm_data(bb["north"], bb["south"], bb["west"], bb["east"])
        print(f"  {len(osm_data['nodes'])} nodes, {len(osm_data['ways'])} ways")

        # --- Build network ---
        network = RoadNetwork.from_osm_data(osm_data, bb["north"], bb["south"], bb["west"], bb["east"])

    # --- Car with AI driver on random road ---
    rx, ry, rh, seg_idx, node_id = network.random_road_point()
    driver = AIDriver()  # Start in RAILS mode
    car = Car(rx, ry, rh, seg_idx, driver)
    car.progress = 0.5
    # Apply the normal right-lane offset immediately (matching what
    # continuous driving would show), instead of leaving the car exactly
    # on the raw centerline - otherwise the very first physics frame
    # "snaps" it sideways into its lane, which the (correctly strict)
    # teleportation watchdog flags as a real jump. Same fix as
    # Car.teleport_random()/teleport_to_named_point().
    _spawn_seg = network.segments[seg_idx]
    _spawn_dx, _spawn_dy = _spawn_seg.x2 - _spawn_seg.x1, _spawn_seg.y2 - _spawn_seg.y1
    _spawn_seg_heading = math.degrees(math.atan2(_spawn_dx, _spawn_dy))
    car.forward = abs((rh - _spawn_seg_heading + 180) % 360 - 180) < 90
    car._apply_plain_segment_position(_spawn_seg)
    
    # --- Physics validator (can be toggled with V key) ---
    validator = PhysicsValidator(enabled=True)
    print("Physics validator: ENABLED (press V to toggle)")
    
    # --- REST API (optional, enable with --api flag) ---
    api = None
    if "--api" in sys.argv:
        api = GameAPI()
        api.start(port=5000)
        # Publish named start points (synthetic test maps only; empty for OSM data)
        api.update_state({
            'start_points': {
                name: {'x': x, 'y': y, 'heading': h, 'segment': seg, 'forward': fwd}
                for name, (x, y, h, seg, fwd) in network.start_points.items()
            }
        })
    elif not smoke_test_frames:
        print("\n💡 Tip: Run with --api to enable REST API for remote control")
        print("   Example: python -m src.main --api")
        print("   Then: curl http://localhost:5000/state\n")

    # --- Camera + Renderer ---
    camera = Camera(WINDOW_WIDTH, WINDOW_HEIGHT)
    camera.x, camera.y = car.x, car.y   # snap to car at start
    renderer = Renderer(network, camera)
    renderer.hud_label = None  # optional short text (e.g. "2/3") set via API /label

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
            elif event.type == pygame.MOUSEBUTTONDOWN:
                camera.handle_mouse_down(event.button, event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                camera.handle_mouse_up(event.button)
            elif event.type == pygame.MOUSEMOTION:
                camera.handle_mouse_motion(event.pos)

        keys = pygame.key.get_pressed()

        # Zoom with +/- keys
        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:
            camera.zoom_in()
        if keys[pygame.K_MINUS]:
            camera.zoom_out()

        # Toggle driver mode with TAB
        if keys[pygame.K_TAB] and not hasattr(main, '_last_tab'):
            main._last_tab = False
        if keys[pygame.K_TAB] and not main._last_tab:
            if isinstance(car.driver, AIDriver):
                car.driver = KeyboardDriver()
            else:
                car.driver = AIDriver()
                car.snap_to_road(network)
                car.target_speed = car.speed
            print(f"Driving mode: {car.driver.get_name()}")
        main._last_tab = keys[pygame.K_TAB]
        
        # Toggle breadcrumb trail with B
        if keys[pygame.K_b] and not hasattr(main, '_last_b'):
            main._last_b = False
        if keys[pygame.K_b] and not main._last_b:
            car.trail_enabled = not car.trail_enabled
            print(f"Breadcrumb trail: {'ON' if car.trail_enabled else 'OFF'}")
        main._last_b = keys[pygame.K_b]
        
        # Random location with R
        if keys[pygame.K_r] and not hasattr(main, '_last_r'):
            main._last_r = False
        if keys[pygame.K_r] and not main._last_r:
            car.teleport_random(network)
            validator.reset_car_state(car)
            print(f"\n🎲 Random location: Segment {car.seg_idx}, Pos ({car.x:.0f}, {car.y:.0f})\n")
        main._last_r = keys[pygame.K_r]
        
        # Toggle physics validator with V
        if keys[pygame.K_v] and not hasattr(main, '_last_v'):
            main._last_v = False
        if keys[pygame.K_v] and not main._last_v:
            if validator.enabled:
                validator.disable()
            else:
                validator.enable()
        main._last_v = keys[pygame.K_v]
        
        # Snap camera to car with 'C' key
        if keys[pygame.K_c]:
            camera.snap_to(car.x, car.y, network.world_width, network.world_height)
        
        # ESC: emergency stop + screenshot to clipboard (for bug reports)
        if keys[pygame.K_ESCAPE] and not hasattr(main, '_last_esc'):
            main._last_esc = False
        esc_pressed_now = keys[pygame.K_ESCAPE] and not main._last_esc
        if esc_pressed_now:
            car.speed = 0.0
            car.target_speed = 0.0
            car.active_turn = None
            print("\n🛑 ESC: Emergency stop\n")
        main._last_esc = keys[pygame.K_ESCAPE]

        # In smoke test, simulate driving inputs
        if smoke_test_frames:
            keys = _FakeKeys(accel=True, right=(frame > smoke_test_frames // 2))
        
        # Handle API commands (if API enabled)
        if api:
            commands = api.get_commands()
            
            # Teleport command
            if 'teleport' in commands:
                teleport_params = commands['teleport']
                if teleport_params.get('start_point'):
                    name = teleport_params['start_point']
                    try:
                        car.teleport_to_named_point(network, name)
                        validator.reset_car_state(car)
                        print(f"\nAPI: Teleport to named start point '{name}' "
                              f"(segment {car.seg_idx}, heading {car.heading:.1f}°)\n")
                    except KeyError as e:
                        print(f"\nAPI: {e}\n")
                elif teleport_params.get('random'):
                    car.teleport_random(network)
                    validator.reset_car_state(car)
                    print(f"\nAPI: Random teleport to segment {car.seg_idx}\n")
                # TODO: Handle specific segment/progress teleport
            
            # Toggle command
            if 'toggle' in commands:
                toggle_params = commands['toggle']
                if 'breadcrumbs' in toggle_params:
                    car.trail_enabled = toggle_params['breadcrumbs']
                    print(f"API: Breadcrumbs {'ON' if car.trail_enabled else 'OFF'}")
                if 'validator' in toggle_params:
                    if toggle_params['validator']:
                        validator.enable()
                    else:
                        validator.disable()
                if 'mode' in toggle_params:
                    mode = toggle_params['mode']
                    if mode == 'rails' and not isinstance(car.driver, AIDriver):
                        car.driver = AIDriver()
                        car.snap_to_road(network)
                        print("API: Switched to RAILS mode")
                    elif mode == 'free' and not isinstance(car.driver, KeyboardDriver):
                        car.driver = KeyboardDriver()
                        print("API: Switched to FREE mode")
            
            # Label command (short HUD text, e.g. "2/3" for a test's map tile)
            if 'label' in commands:
                renderer.hud_label = commands['label']

        # Get control input from driver (keyboard or API)
        control_input = car.driver.get_control(car, network, dt, keys)
        
        # Merge API control inputs (if API enabled)
        if api:
            api_control = api.get_control()
            # API control overrides keyboard for specific keys
            if api_control['accelerate']:
                control_input['accelerate'] = True
            if api_control['brake']:
                control_input['brake'] = True
            if api_control['steer_left']:
                control_input['steer_left'] = True
            if api_control['steer_right']:
                control_input['steer_right'] = True
            if api_control['blinker_left']:
                control_input['blinker_left'] = True
            if api_control['blinker_right']:
                control_input['blinker_right'] = True
            
            # The AIDriver's actual turn CHOICE (which segment to take at
            # a junction) comes from its own `pending_turn` attribute -
            # which get_control() above only ever sets from real keyboard
            # key-edge-detection (K_LEFT/K_a, K_RIGHT/K_d). Merging the
            # API's blinker flags into control_input alone only affected
            # cosmetic blinker-light rendering elsewhere; it never told
            # the driver which way to actually turn, so API-controlled
            # turns silently fell back to whatever `pending_turn` already
            # was (usually None -> "straight", or whatever the keyboard
            # last set) regardless of which blinker the API requested.
            # Explicitly sync pending_turn/blinker state from the API's
            # request here so a remote-controlled turn actually steers
            # the requested direction at the next junction.
            if hasattr(car.driver, 'pending_turn'):
                if api_control['blinker_left']:
                    car.driver.pending_turn = 'left'
                    car.driver.blinker_left = True
                    car.driver.blinker_right = False
                elif api_control['blinker_right']:
                    car.driver.pending_turn = 'right'
                    car.driver.blinker_right = True
                    car.driver.blinker_left = False
                elif not (keys[pygame.K_LEFT] or keys[pygame.K_a] or
                          keys[pygame.K_RIGHT] or keys[pygame.K_d]):
                    # Neither the API nor the keyboard is requesting a
                    # turn right now - make sure a previous API-driven
                    # blinker doesn't stay stuck on.
                    car.driver.pending_turn = None
                    car.driver.blinker_left = False
                    car.driver.blinker_right = False
        
        # Update car physics
        car.update(dt, network, control_input)
        
        # Run physics validation (independent check)
        validator.check(car, dt, network)

        # Off-road check (FREE mode only)
        if car.driver.get_name() == "FREE":
            if not car.is_on_road(network):
                car.speed = 0
            # Map edge check
            bounds = network.bounds
            if car.x < 0 or car.x > bounds[2] or car.y < 0 or car.y > bounds[3]:
                car.speed = 0
                car.x = max(0, min(bounds[2], car.x))
                car.y = max(0, min(bounds[3], car.y))

        # Camera follow (only when moving)
        camera.update(car.x, car.y, network.world_width, network.world_height,
                      follow=car.speed > 0.1)

        # Render
        screen.fill(BG_COLOR)
        renderer.draw(screen, car)
        car.draw(screen, camera)

        pygame.display.flip()
        
        # ESC was just pressed: capture this frame (car now stopped) and
        # copy it to the system clipboard for easy bug reporting
        if esc_pressed_now:
            copy_screenshot_to_clipboard(screen)
        
        # Update API state and screenshot (if API enabled)
        if api:
            api.update_state({
                'frame': frame,
                'time': frame * dt_fixed if smoke_test_frames else frame / 60.0,
                'x': car.x,
                'y': car.y,
                'heading': car.heading,
                'speed': car.speed,
                'speed_kmh': car.speed * 3.6,
                'segment': car.seg_idx,
                'progress': car.progress,
                'forward': car.forward,
                'on_road': car.is_on_road(network),
                'driver': car.driver.get_name(),
                'trail_enabled': car.trail_enabled,
                'validator_enabled': validator.enabled,
                'camera_x': camera.x,
                'camera_y': camera.y,
                'camera_zoom': camera.zoom,
            })
            
            # Update screenshot (every 10 frames to reduce overhead)
            if frame % 10 == 0 and not smoke_test_frames:
                # Convert surface to PNG bytes
                import io
                png_io = io.BytesIO()
                pygame.image.save(screen, png_io)
                png_io.seek(0)
                api.update_screenshot(png_io.read())

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
