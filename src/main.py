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
from .driver import Driver, KeyboardDriver, BicycleDriver
from .physics_validator import PhysicsValidator
from .lane_guard import LaneGuard
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


def _distance_to_junction(car, network) -> float:
    """Metres from the car to the junction it is heading toward (for the API)."""
    seg = network.segments[car.seg_idx]
    return ((1.0 - car.progress) if car.forward else car.progress) * seg.length


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

    def _create_car(start_point=None):
        """Create a fresh Car at a random road point or named start.

        Places the car flush against the right kerb (right-hand traffic)
        and a short way INTO the segment, never exactly on the node - a
        node sits in the middle of the junction rounding, where the lane
        geometry is ambiguous and the reference line is still curving.
        """
        if start_point:
            rx, ry, rh, seg_idx, fwd = network.get_start_point(start_point)
        else:
            rx, ry, rh, seg_idx, _ = network.random_road_point()

        _seg = network.segments[seg_idx]
        rad = math.radians(rh)

        # Sit at the kerb. Shared with BicycleNav's pull-over / pull-out
        # target via config.kerb_offset_m() so the two cannot drift apart.
        # (The lateral offset is the same for every point on the car's
        # centreline, so offsetting the rear axle offsets the whole flank.)
        offset_m = kerb_offset_m(_seg.width)
        rx += math.cos(rad) * offset_m * PIXELS_PER_METER
        ry -= math.sin(rad) * offset_m * PIXELS_PER_METER

        # Advance a fraction of the way in, so short segments work too.
        progress0 = SPAWN_PROGRESS if start_point else 0.5
        if start_point:
            advance_m = progress0 * _seg.length
            rx += math.sin(rad) * advance_m * PIXELS_PER_METER
            ry += math.cos(rad) * advance_m * PIXELS_PER_METER

        car = Car(rx, ry, rh, seg_idx, BicycleDriver())
        # progress runs start_node -> end_node, so a car travelling
        # backwards along the segment starts near 1.0, not near 0.0.
        car.progress = (progress0 if fwd else 1.0 - progress0) if start_point else 0.5
        car.forward = fwd if start_point else None
        if not start_point:
            _dx, _dy = _seg.x2 - _seg.x1, _seg.y2 - _seg.y1
            _seg_h = math.degrees(math.atan2(_dx, _dy))
            car.forward = abs((rh - _seg_h + 180) % 360 - 180) < 90
        # Breadcrumbs on from the start: the tyre tracks are the main way
        # to see what the car actually did on its way here.
        car.trail_enabled = True
        return car

    # --- Car with AI driver ---
    # Prefer a deterministic spawn: a random road point means every run
    # starts somewhere different, so nothing before the first teleport is
    # reproducible (and a crash on startup is a different crash each time).
    # --start <name> picks a named point; otherwise take the first one the
    # map defines, falling back to random only for maps with none (real OSM).
    start_name = None
    if "--start" in sys.argv:
        idx = sys.argv.index("--start")
        if idx + 1 < len(sys.argv):
            start_name = sys.argv[idx + 1]
    if start_name is None and network.start_points:
        start_name = sorted(network.start_points)[0]
    car = _create_car(start_name)
    if start_name:
        print(f"Spawn: '{start_name}' (deterministic; --start <name> to change)")
    print("Navigation model: BICYCLE (kinematic, free particle)")
    
    # --- Physics validator (can be toggled with V key) ---
    validator = PhysicsValidator(enabled=True)
    print("Physics validator: ENABLED (press V to toggle)")
    
    # --- Lane guard (wrong-side detection, softer than validator) ---
    lane_guard = LaneGuard(enabled=True)
    lenient = "--lenient" in sys.argv
    print(f"Lane guard: ENABLED{' (lenient - wrong-side warns, does not stop)' if lenient else ''}")
    
    # --- REST API (optional, enable with --api flag) ---
    api = None
    if "--api" in sys.argv:
        api_port = 5000
        if "--port" in sys.argv:
            api_port = int(sys.argv[sys.argv.index("--port") + 1])
        api = GameAPI()
        api.start(port=api_port)
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
    if map_name:
        # Start with the whole synthetic test map visible (zoom fitted
        # to the world, never closer than 1x).
        fit = min(WINDOW_WIDTH / network.world_width, WINDOW_HEIGHT / network.world_height)
        camera.zoom = max(MIN_ZOOM, min(fit, 1.0))
    renderer = Renderer(network, camera)
    renderer.hud_label = None  # optional short text (e.g. "2/3") set via API /label

    # --- Game loop ---
    frame = 0
    running = True
    frozen = False
    dt_fixed = 1 / 60
    while running:
        frame += 1
        if smoke_test_frames and frame > smoke_test_frames:
            running = False
            break

        dt = clock.tick(60) / 1000.0 if not smoke_test_frames else dt_fixed
        # Clamp the physics step. If a frame stalls (route rebuild, GC, the
        # window being dragged), the real elapsed time can be hundreds of
        # milliseconds; integrating that in one go teleports the car metres
        # downroad, straight through any geometry in between. Better to run
        # briefly in slow motion than to take an unphysical leap.
        dt = min(dt, 1.0 / 30.0)

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
            if isinstance(car.driver, BicycleDriver):
                car.driver = KeyboardDriver()
            else:
                car.driver = BicycleDriver()
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
        
        # Random location with R (destroy old car, create new one)
        if keys[pygame.K_r] and not hasattr(main, '_last_r'):
            main._last_r = False
        if keys[pygame.K_r] and not main._last_r:
            car = _create_car()
            camera.snap_to(car.x, car.y, network.world_width, network.world_height)
            print(f"\n🎲 New car at segment {car.seg_idx}, "
                  f"Pos ({car.x:.0f}, {car.y:.0f})\n")
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
        
        # ESC: toggle freeze (pause game loop, take screenshot)
        if keys[pygame.K_ESCAPE] and not hasattr(main, '_last_esc'):
            main._last_esc = False
        esc_pressed_now = keys[pygame.K_ESCAPE] and not main._last_esc
        if esc_pressed_now:
            frozen = not frozen
            if frozen:
                print("\n⏸️  ESC: Game frozen (press ESC to resume)\n")
            else:
                print("▶️  ESC: Game resumed\n")
        main._last_esc = keys[pygame.K_ESCAPE]

        # In smoke test, simulate driving inputs
        if smoke_test_frames:
            keys = _FakeKeys(accel=True, right=(frame > smoke_test_frames // 2))
        
        # Handle API commands (if API enabled)
        if api:
            commands = api.get_commands()
            
            # Replace-car command (destroy old car, create fresh one)
            if 'teleport' in commands:
                tp = commands['teleport']
                start_point = tp.get('start_point') or None
                car = _create_car(start_point)
                camera.snap_to(car.x, car.y, network.world_width, network.world_height)
                print(f"\n🔄 New car at segment {car.seg_idx}, "
                      f"heading {car.heading:.1f}°\n")
            
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
                    if mode == 'bicycle' and not isinstance(car.driver, BicycleDriver):
                        car.driver = BicycleDriver()
                        car.bicycle_nav = None
                        print("API: Switched to BICYCLE mode")
                    elif mode == 'free' and not isinstance(car.driver, KeyboardDriver):
                        car.driver = KeyboardDriver()
                        print("API: Switched to FREE mode")
            
            # Label command (short HUD text, e.g. "2/3" for a test's map tile)
            if 'label' in commands:
                renderer.hud_label = commands['label']

        # Skip physics when frozen
        if not frozen:
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
            
            # BicycleDriver's turn choice comes from `pending_turn`.
            # API blinkers are ONE-SHOT commands (like flicking a real
            # indicator): apply them once, then consume the flag so they
            # don't re-trigger every frame. The driver's own blinker state
            # persists afterwards and is cleared by the car itself once the
            # turn is executed (see clear_blinker_if_turned below).
            if hasattr(car.driver, 'pending_turn'):
                if api_control['blinker_left']:
                    car.driver.pending_turn = 'left'
                    car.driver.blinker_left = True
                    car.driver.blinker_right = False
                    api.clear_control('blinker_left')
                elif api_control['blinker_right']:
                    car.driver.pending_turn = 'right'
                    car.driver.blinker_right = True
                    car.driver.blinker_left = False
                    api.clear_control('blinker_right')
        
        # Update car physics (only when not frozen)
        if not frozen:
            prev_seg = car.seg_idx
            car.update(dt, network, control_input)
        
        # Auto-cancel the blinker once the turn is actually executed
        # (like a real car: the indicator switches off by itself after the
        # turn - this is the car's job, not the test's).
        if car.seg_idx != prev_seg and hasattr(car.driver, 'clear_blinker_if_turned'):
            car.driver.clear_blinker_if_turned(car, network, prev_seg, car.seg_idx)
        
        # Run physics validation (independent check)
        validator.check(car, dt, network)
        
        # Lane guard: check for wrong-side driving (skip during active
        # turns — lateral offset from incoming centerline is expected)
        in_turn = hasattr(car.bicycle_nav, '_s') and car.bicycle_nav._s is not None and \
                  car.bicycle_nav._in_turn_blend_zone(car.bicycle_nav._s)
        on_wrong_side = False
        if not in_turn:
            on_wrong_side = lane_guard.check(car, dt, network)
            if on_wrong_side and not lenient:
                # Fatal by default so the test suite cannot silently pass a
                # run that broke rule 2. Pass --lenient to explore a map
                # instead: the guard still reports and the HUD still blinks,
                # but the process survives. (LaneGuard itself never raises -
                # this escalation is the game loop's choice.)
                raise RuntimeError(
                    f"WRONG-SIDE DRIVING! Car at ({car.x:.1f}, {car.y:.1f}), "
                    f"heading {car.heading:.1f}°, segment {car.seg_idx}"
                )

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

        # Camera follow (only when moving and not frozen)
        if not frozen:
            camera.update(car.x, car.y, network.world_width, network.world_height,
                          follow=car.speed > 0.1)

        # Render
        screen.fill(BG_COLOR)  # solid grass-green background
        renderer.draw(screen, car)
        car.draw(screen, camera)
        renderer.draw_trail(screen, car)  # after sprite so buckets are visible
        
        # HUD: wrong-side warning LED (blinks red when on opposing lane)
        if on_wrong_side:
            # Blink at ~3 Hz (on for 10 frames, off for 10 frames at 60 fps)
            if frame % 20 < 15:
                wx = WINDOW_WIDTH - 50
                wy = WINDOW_HEIGHT - 50
                pygame.draw.circle(surface=screen, color=(255, 30, 30), center=(wx, wy), radius=14)
                pygame.draw.circle(surface=screen, color=(255, 255, 255), center=(wx, wy), radius=3)

        pygame.display.flip()
        
        # On freeze: capture this frame and copy to clipboard
        if esc_pressed_now and frozen:
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
                'distance_to_junction': _distance_to_junction(car, network),
                'on_road': car.is_on_road(network),
                'driver': car.driver.get_name(),
                'trail_enabled': car.trail_enabled,
                'validator_enabled': validator.enabled,
                'validator_violations': len(validator.violations),
                'lane_guard_stats': lane_guard.stats(),
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
