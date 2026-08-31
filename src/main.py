#!/usr/bin/env python3
"""Car Game — Entry Point"""

import math
import os
import sys
import time
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
from .obstacles import ObstacleManager
from .obstacle_ui import ObstaclePalette
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


def _flag_position_on_route(network, nav, seg_idx: int,
                            progress: float) -> list | None:
    """World position + travel heading at `progress` along segment `seg_idx`,
    following the direction the CURRENT ROUTE traverses that segment.

    The route is a node path; consecutive nodes (a, b) define the traversal
    direction of the segment between them. This keeps the flag on the right
    side of the road even if the scenario drives the segment backward.
    Returns [x_px, y_px, heading_deg] or None if the segment isn't in the
    route's node path.
    """
    route = getattr(nav, '_route', None) or []
    seg = network.segments[seg_idx]
    for i in range(len(route) - 1):
        a, b = route[i], route[i + 1]
        if a != b and a in (seg.start_node, seg.end_node) \
                and b in (seg.start_node, seg.end_node):
            ax, ay = network.nodes[a]
            bx, by = network.nodes[b]
            x = ax + (bx - ax) * progress
            y = ay + (by - ay) * progress
            h = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
            return [x, y, h]
    return None


def main(smoke_test_frames: int = 0):
    """Run the game. If smoke_test_frames > 0, run headless for that many frames."""
    # --- Init ---
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT),
                                     pygame.RESIZABLE)
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

    # --- Obstacles (docs/OBSTACLES.md) ---
    # Static parked cars: palette UI (top-right, left mouse) + REST API go
    # through the SAME manager/placement logic. Layouts save per map name.
    map_label = map_name or "kleinmachnow"
    obstacle_mgr = ObstacleManager(map_label)

    def _spawn_position(start_point, progress=None):
        """World position/heading where a car spawned at `start_point` will
        sit: normal driving position (right-lane centre) plus the start
        point's lateral offset plus `progress` (0..1) along the start
        segment from the named node. Shared by _create_car and the --start
        camera focus so both agree on exactly where the car will be."""
        rx, ry, rh, seg_idx, fwd, lat_off = network.get_start_point(start_point)
        node_x, node_y = rx, ry   # the named (degree-1) node itself
        _seg = network.segments[seg_idx]
        rad = math.radians(rh)

        # Sit in the NORMAL DRIVING POSITION (right-lane centre), not at
        # the kerb: e2e scenarios start in traffic, not parked - a curb
        # spawn cost every test a ~2 s pull-out before it could even
        # accelerate (decision 2026-08-27: run the tests faster). The
        # start point's lateral_offset_m shifts it toward the right kerb
        # (+) / the left side of the road (-). The offset is returned so
        # _create_car can hand it to the nav (the car must HOLD this line,
        # not re-center onto the nominal lane).
        # Road-aware normal position: on multi-lane carriageways that is
        # the centre of the outermost DRIVING lane (next to the parking
        # lane), not a fixed 1.75 m - see config.lane_base_offset_m.
        offset_m = lane_base_offset_m(_seg.width, _seg.lanes,
                                      _seg.parking_lane_width, _seg.oneway) \
            + lat_off
        rx += math.cos(rad) * offset_m * PIXELS_PER_METER
        ry -= math.sin(rad) * offset_m * PIXELS_PER_METER

        # Advance a fraction of the way in, so short segments work too.
        progress0 = progress if progress is not None else SPAWN_PROGRESS
        advance_m = progress0 * _seg.length
        # The chord above is only the segment's LOGICAL connection. Where
        # the road actually runs - the corner-rounded centreline the
        # pavement is built from - can diverge from it by tens of metres
        # near sharp corners (a 166-deg hairpin with the 6 m curb radius
        # starts curving ~49 m before the node), and a chord-based spawn
        # there sits off the pavement. Place on the real path instead -
        # walking it from the entry node, then offsetting into the lane;
        # on straight segments the two agree exactly.
        rounded = network.spawn_path_point(seg_idx, (node_x, node_y), advance_m)
        if rounded is not None:
            rx, ry, rh = rounded
            rad = math.radians(rh)   # local tangent, not the chord heading
            rx += math.cos(rad) * offset_m * PIXELS_PER_METER
            ry -= math.sin(rad) * offset_m * PIXELS_PER_METER
        else:
            rx += math.sin(rad) * advance_m * PIXELS_PER_METER
            ry += math.cos(rad) * advance_m * PIXELS_PER_METER
        return rx, ry, rh, seg_idx, fwd, offset_m

    def _create_car(start_point=None, progress=None):
        """Create a fresh Car at a random road point or named start.

        `progress` (0..1) places the car at that fraction along the start
        segment from the named node; e2e tests use 0.5 so every scenario
        starts mid-segment instead of hugging the junction node.

        Places the car flush against the right kerb (right-hand traffic)
        and a short way INTO the segment, never exactly on the node - a
        node sits in the middle of the junction rounding, where the lane
        geometry is ambiguous and the reference line is still curving.
        """
        if start_point:
            rx, ry, rh, seg_idx, fwd, spawn_offset_m = \
                _spawn_position(start_point, progress)
        else:
            rx, ry, rh, seg_idx, _ = network.random_road_point()
            spawn_offset_m = None

        car = Car(rx, ry, rh, seg_idx, BicycleDriver())
        # progress runs start_node -> end_node, so a car travelling
        # backwards along the segment starts near 1.0, not near 0.0.
        if start_point:
            progress0 = progress if progress is not None else SPAWN_PROGRESS
            car.progress = progress0 if fwd else 1.0 - progress0
            car.forward = fwd
            # The nav's nominal lane offset follows the SPAWN position, so
            # the car holds its initial lateral line up to the flag and
            # parks from it (docs §1 variant) instead of re-centering.
            car.lane_offset_override_m = spawn_offset_m
        else:
            car.progress = 0.5
        if not start_point:
            _seg = network.segments[seg_idx]
            _dx, _dy = _seg.x2 - _seg.x1, _seg.y2 - _seg.y1
            _seg_h = math.degrees(math.atan2(_dx, _dy))
            car.forward = abs((rh - _seg_h + 180) % 360 - 180) < 90
        # Breadcrumbs on from the start: the tyre tracks are the main way
        # to see what the car actually did on its way here.
        car.trail_enabled = True
        return car

    # --- Car with AI driver ---
    # Synthetic test maps do NOT auto-spawn a car: the e2e suite teleports
    # its own car in (POST /teleport), and an idle AI car driving around
    # before the first teleport is noise that makes every run less
    # reproducible. On a test map, --start <name> focuses the CAMERA on
    # that start point (the suite's car then appears exactly there); real
    # OSM data keeps its random spawn.
    start_name = None
    if "--start" in sys.argv:
        idx = sys.argv.index("--start")
        if idx + 1 < len(sys.argv):
            start_name = sys.argv[idx + 1]

    focus_point = None   # (x, y) world position for the initial camera view
    if map_name:
        car = None
        if start_name is not None:
            if start_name in network.start_points:
                # Focus EXACTLY where the e2e suite's teleport will put the
                # car (progress 0.5, same kerb offset as _create_car), so
                # the view never has to jump when it appears.
                _fx, _fy = _spawn_position(start_name, 0.5)[0:2]
                focus_point = (_fx, _fy)
                print(f"Camera focused at '{start_name}' "
                      f"(no car - the e2e suite teleports one in)")
            else:
                print(f"⚠️  Unknown start point '{start_name}' - ignoring it")
        else:
            print("No car spawned (test map - the e2e suite teleports one in; "
                  "use --start <name> to focus the camera, or POST /teleport)")
    else:
        car = _create_car(start_name)   # real OSM data: random spawn as before
        if start_name:
            print(f"Spawn: '{start_name}' (deterministic; --start <name> to change)")
    print("Navigation model: BICYCLE (kinematic, free particle)")
    
    # --- Physics validator (can be toggled with V key) ---
    validator = PhysicsValidator(enabled=True)
    print("Physics validator: ENABLED (press V to toggle)")
    
    # --- Lane guard (wrong-side detection, softer than validator) ---
    lane_guard = LaneGuard(enabled=True)
    print("Lane guard: ENABLED (wrong-side is a warning here - the e2e "
          "suite turns it into a test failure)")
    
    # --- REST API (optional, enable with --api flag) ---
    api = None
    if "--api" in sys.argv:
        api_port = 5000
        if "--port" in sys.argv:
            api_port = int(sys.argv[sys.argv.index("--port") + 1])
        api = GameAPI()
        api.start(port=api_port)
        api.set_obstacles(obstacle_mgr, network)
        # Publish named start points (synthetic test maps only; empty for OSM data)
        api.update_state({
            'start_points': {
                name: {'x': x, 'y': y, 'heading': h, 'segment': seg,
                       'forward': fwd, 'lateral_offset_m': off}
                for name, (x, y, h, seg, fwd, off) in network.start_points.items()
            }
        })
    elif not smoke_test_frames:
        print("\n💡 Tip: Run with --api to enable REST API for remote control")
        print("   Example: python -m src.main --api")
        print("   Then: curl http://localhost:5000/state\n")

    # --- Camera + Renderer ---
    camera = Camera(WINDOW_WIDTH, WINDOW_HEIGHT)
    if car is not None:
        camera.x, camera.y = car.x, car.y   # snap to car at start
    if map_name:
        # Test maps open at a close driving zoom (7x), focused on the
        # --start point when given, else on the world centre - NOT fitted
        # to show the whole map: you want to watch the maneuver up close.
        # The suite's teleport then places the car exactly where the view
        # is looking. The minimap keeps the overview; '-' / mouse wheel
        # zooms out to see the full network.
        camera.zoom = 7.0
        if focus_point is not None:
            camera.x, camera.y = focus_point
        else:
            camera.x, camera.y = network.world_width / 2, network.world_height / 2
    renderer = Renderer(network, camera)
    renderer.hud_label = None  # optional short text (e.g. "2/3") set via API /label
    obstacle_ui = ObstaclePalette(obstacle_mgr, network, camera, renderer)

    # --- Game loop ---
    # Quit button (top-left, drawn topmost): an orderly way to end the game
    # so a closed window is never mistaken for a crash by the e2e suite -
    # every deliberate exit prints "orderly shutdown" to the log.
    _quit_rect = pygame.Rect(10, 10, 64, 28)
    frame = 0
    running = True
    frozen = False
    dt_fixed = 1 / 60
    _physics_accum = 0.0
    _prev_render = None   # (x, y, heading) before the last physics substep
    while running:
        frame += 1
        if smoke_test_frames and frame > smoke_test_frames:
            running = False
            break

        frame_ms = clock.tick(60) if not smoke_test_frames else 1000.0 / 60
        dt = frame_ms / 1000.0
        # Clamp the physics step. If a frame stalls (route rebuild, GC, the
        # window being dragged), the real elapsed time can be hundreds of
        # milliseconds; integrating that in one go teleports the car metres
        # downroad, straight through any geometry in between. Better to run
        # briefly in slow motion than to take an unphysical leap.
        dt = min(dt, 1.0 / 30.0)

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("\n👋 Window closed - orderly shutdown (NOT a crash).")
                running = False
            elif event.type == pygame.VIDEORESIZE:
                # Resizable window: rebuild the surface and tell the camera
                # its new size (the renderer already uses live surface
                # dimensions for everything it draws).
                screen = pygame.display.set_mode((event.w, event.h),
                                                 pygame.RESIZABLE)
                camera.width, camera.height = event.w, event.h
            elif event.type == pygame.MOUSEWHEEL:
                camera.handle_zoom(event.y)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # The Quit button gets first claim; then the obstacle palette
                # (slots, world obstacles); what both decline - empty map area
                # - starts a camera pan drag.
                if event.button == 1 and _quit_rect.collidepoint(event.pos):
                    print("\n👋 Quit button pressed - orderly shutdown "
                          "(NOT a crash).")
                    running = False
                elif not obstacle_ui.handle_event(event):
                    camera.handle_mouse_down(event.button, event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                if not obstacle_ui.handle_event(event):
                    camera.handle_mouse_up(event.button)
            elif event.type == pygame.MOUSEMOTION:
                obstacle_ui.handle_event(event)
                camera.handle_mouse_motion(event.pos)
            elif event.type == pygame.KEYDOWN:
                # Consumed while a layout name is being typed (ESC/ENTER/
                # BACKSPACE must not trigger the game's shortcuts).
                obstacle_ui.handle_keydown(event)
            elif event.type == pygame.TEXTINPUT:
                obstacle_ui.handle_textinput(event)

        keys = pygame.key.get_pressed()

        # While a layout name is being typed, ALL game keyboard shortcuts are
        # suspended (typing 'b' must not toggle the breadcrumb trail, 'r' must
        # not teleport a new car, ESC must not freeze). When typing ends, the
        # edge-detect flags are reset so a still-held key cannot fire.
        _typing = obstacle_ui.text_input_active()
        if getattr(main, '_was_typing', False) and not _typing:
            for _attr in ('_last_tab', '_last_b', '_last_r', '_last_v',
                          '_last_u', '_last_esc'):
                if hasattr(main, _attr):
                    delattr(main, _attr)
        main._was_typing = _typing

        # Zoom with +/- keys
        if not _typing and (keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]):
            camera.zoom_in()
        if not _typing and keys[pygame.K_MINUS]:
            camera.zoom_out()

        # Toggle driver mode with TAB
        if _typing:
            main._last_tab = keys[pygame.K_TAB]
        elif keys[pygame.K_TAB] and not hasattr(main, '_last_tab'):
            main._last_tab = False
        if keys[pygame.K_TAB] and not main._last_tab and car is not None and not _typing:
            if isinstance(car.driver, BicycleDriver):
                car.driver = KeyboardDriver()
            else:
                car.driver = BicycleDriver()
                car.snap_to_road(network)
                car.target_speed = car.speed
            print(f"Driving mode: {car.driver.get_name()}")
        main._last_tab = keys[pygame.K_TAB]
        
        # Toggle breadcrumb trail with B
        if _typing:
            main._last_b = keys[pygame.K_b]
        elif keys[pygame.K_b] and not hasattr(main, '_last_b'):
            main._last_b = False
        if keys[pygame.K_b] and not main._last_b and car is not None and not _typing:
            car.trail_enabled = not car.trail_enabled
            print(f"Breadcrumb trail: {'ON' if car.trail_enabled else 'OFF'}")
        main._last_b = keys[pygame.K_b]
        
        # Random location with R (destroy old car, create new one)
        if _typing:
            main._last_r = keys[pygame.K_r]
        elif keys[pygame.K_r] and not hasattr(main, '_last_r'):
            main._last_r = False
        if keys[pygame.K_r] and not main._last_r and not _typing:
            car = _create_car()
            camera.snap_to(car.x, car.y, network.world_width, network.world_height)
            print(f"\n🎲 New car at segment {car.seg_idx}, "
                  f"Pos ({car.x:.0f}, {car.y:.0f})\n")
        main._last_r = keys[pygame.K_r]
        
        # Toggle physics validator with V
        if _typing:
            main._last_v = keys[pygame.K_v]
        elif keys[pygame.K_v] and not hasattr(main, '_last_v'):
            main._last_v = False
        if keys[pygame.K_v] and not main._last_v and not _typing:
            if validator.enabled:
                validator.disable()
            else:
                validator.enable()
        main._last_v = keys[pygame.K_v]
        
        # Paved-edge debug outline with G: white line along the exact
        # paved-area polygon that defines the off-road check, so you can
        # see where the game THINKS the road edge is (vs. what is drawn).
        if _typing:
            main._last_g = keys[pygame.K_g]
        elif keys[pygame.K_g] and not hasattr(main, '_last_g'):
            main._last_g = False
        if keys[pygame.K_g] and not main._last_g and not _typing:
            renderer.set_paved_edge(not renderer.paved_edge_visible)
            print(f"Paved edge outline: "
                  f"{'ON' if renderer.paved_edge_visible else 'OFF'}")
        main._last_g = keys[pygame.K_g]
        
        # U-turn (Wenden) with 'U' - one-shot request to the nav
        if _typing:
            main._last_u = keys[pygame.K_u]
        elif keys[pygame.K_u] and not hasattr(main, '_last_u'):
            main._last_u = False
        if keys[pygame.K_u] and not main._last_u and car is not None and not _typing:
            if isinstance(car.driver, BicycleDriver):
                car.driver.uturn_requested = True
                print("\n🔄 U-turn (Wenden) requested\n")
        main._last_u = keys[pygame.K_u]

        # Snap camera to car with 'C' key
        if not _typing and keys[pygame.K_c] and car is not None:
            camera.snap_to(car.x, car.y, network.world_width, network.world_height)
        
        # ESC: toggle freeze (pause game loop, take screenshot).
        # While typing a layout name, ESC cancels the input instead
        # (consumed by obstacle_ui.handle_keydown above).
        if _typing:
            main._last_esc = keys[pygame.K_ESCAPE]
            esc_pressed_now = False
        else:
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
                car = _create_car(start_point, progress=tp.get('progress'))
                # Optional rolling start (m/s): the running turn tests spawn
                # already moving instead of accelerating from a standstill.
                if tp.get('speed') is not None:
                    car.speed = float(tp['speed'])
                camera.snap_to(car.x, car.y, network.world_width, network.world_height)
                print(f"\n🔄 New car at segment {car.seg_idx}, "
                      f"heading {car.heading:.1f}°"
                      f"{' (rolling start)' if tp.get('speed') is not None else ''}\n")
            
            # Toggle command
            if 'toggle' in commands:
                toggle_params = commands['toggle']
                if 'breadcrumbs' in toggle_params and car is not None:
                    car.trail_enabled = toggle_params['breadcrumbs']
                    print(f"API: Breadcrumbs {'ON' if car.trail_enabled else 'OFF'}")
                if 'validator' in toggle_params:
                    if toggle_params['validator']:
                        validator.enable()
                    else:
                        validator.disable()
                if 'paved_edge' in toggle_params:
                    renderer.set_paved_edge(bool(toggle_params['paved_edge']))
                    print(f"API: Paved edge outline "
                          f"{'ON' if renderer.paved_edge_visible else 'OFF'}")
                if 'mode' in toggle_params and car is not None:
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

            # Test confirmation flags: green at the scenario start
            # (car position, immediate), red at its end given as
            # [segment, progress] - resolved to a map position by the main
            # loop once the route covers that segment, so it is visible
            # from the START of the test.
            if 'flags' in commands and isinstance(commands['flags'], dict):
                fl = commands['flags']
                # Only update the keys that are present, so setting the
                # end flag doesn't wipe the start flag (and vice versa).
                if 'green' in fl:
                    renderer.flag_green = fl['green']
                if 'red' in fl:
                    if fl['red'] is None:
                        renderer.flag_red = None
                        renderer.flag_red_pending = None
                        if car is not None and car.bicycle_nav is not None:
                            car.bicycle_nav.clear_destination()
                    else:
                        seg_idx, prog = fl['red'][0], fl['red'][1]
                        renderer.flag_red = None
                        renderer.flag_red_pending = (seg_idx, prog)
                        # red_nav (default True): the flag becomes the car's
                        # navigation destination (park at it). Running turn
                        # tests send red_nav=False - the flag is a visual
                        # end-of-test marker only (docs/TESTING.md).
                        renderer.flag_red_nav = bool(fl.get('red_nav', True))

            # Hazard lights (Warnblinkanlage): explicit on/off command.
            if 'hazard' in commands and car is not None \
                    and isinstance(car.driver, BicycleDriver):
                car.driver.set_hazard(
                    bool(commands['hazard']),
                    reason="manual (REST API)" if commands['hazard']
                           else "manual off (REST API)")

        # Skip physics when frozen (or while no car is on the map yet)
        if not frozen and car is not None:
            # Get control input from driver (keyboard or API)
            control_input = car.driver.get_control(car, network, dt, keys)
        
        # Merge API control inputs (if API enabled)
        if api and car is not None:
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
            # persists afterwards and is cleared by the car itself via the
            # mechanical auto-off (steered in + steered back = off).
            if hasattr(car.driver, 'signal_turn'):
                if api_control['blinker_left']:
                    car.driver.signal_turn('left')
                    api.clear_control('blinker_left')
                elif api_control['blinker_right']:
                    car.driver.signal_turn('right')
                    api.clear_control('blinker_right')
            # U-turn (Wenden) is a one-shot command, like a blinker.
            if api_control.get('uturn') and hasattr(car.driver, 'uturn_requested'):
                car.driver.uturn_requested = True
                api.clear_control('uturn')
        
        # --- Physics: FIXED-timestep substeps (only when not frozen) ----
        # Integrate at exactly dt_fixed (1/60 s) no matter how long the
        # frame took. Variable dt let render hitches (tile generation,
        # window drags, GC) change the integration steps and shifted
        # cornering trajectories by more than the 0.35 m kerb clearance -
        # the same scenario passed headless but clipped off-road live.
        # Fixed steps make the live game deterministic: if a scenario is
        # clean in a headless run, it is clean here.
        _t_a = time.perf_counter()
        on_wrong_side = False
        free_off_road = False
        if frozen or car is None:
            _physics_accum = 0.0          # don't simulate the pause / empty map
            _steps_this_frame = 0
        else:
            _physics_accum += min(dt, 0.25)
            if _physics_accum > 4 * dt_fixed:
                _physics_accum = 4 * dt_fixed   # slow motion, never a leap
            _steps_this_frame = 0
            while _physics_accum >= dt_fixed:
                _physics_accum -= dt_fixed
                _steps_this_frame += 1
                _prev_render = (car.x, car.y, car.heading)
                _pre_x, _pre_y, _pre_h = car.x, car.y, car.heading
                car.update(dt_fixed, network, control_input)
                # (Blinker auto-cancel is mechanical - the driver watches
                # the steering angle itself: steered in + steered back =
                # off. No segment-change hook needed.)
                # Stop-on-contact with obstacles (ALL modes; docs/OBSTACLES.md):
                # brake at full A_BRAKE and clamp so the body box never
                # interpenetrates an obstacle - the car rests against it.
                in_contact = obstacle_mgr.apply_contact_stop(
                    car, dt_fixed, _pre_x, _pre_y, _pre_h)
                # Physics validation (independent check). While in contact
                # the motion is externally constrained by a solid object, so
                # the validator suspends the turning-radius invariant for
                # that frame - jump/snap/off-road checks keep running.
                validator.check(car, dt_fixed, network, in_contact=in_contact)
                # Lane guard: wrong-side driving (skipped during active
                # turns - lateral offset from the incoming centreline is
                # expected; and for the whole U-turn, where crossing the
                # centreline is INTENDED, spec §5).
                in_turn = hasattr(car.bicycle_nav, '_s') \
                    and car.bicycle_nav._s is not None \
                    and car.bicycle_nav._in_turn_blend_zone(car.bicycle_nav._s)
                uturn_now = car.bicycle_nav is not None and \
                            getattr(car.bicycle_nav, 'uturn_active', False)
                on_wrong_side = False
                if not in_turn and not uturn_now:
                    # Warning only - NEVER fatal here (no crash, no freeze):
                    # the guard prints once per crossing and /state reports
                    # 'wrong_side' + cumulative lane-guard stats; FAILING a
                    # run is the e2e suite's job (docs/TESTING.md §3). In
                    # FREE mode the same flag drives the red LED + label.
                    on_wrong_side = lane_guard.check(car, dt_fixed, network)
                # Off-road check (FREE mode only): stop the car + warning
                free_off_road = False
                if car.driver.get_name() == "FREE":
                    free_off_road = not car.is_on_road(network)
                    if free_off_road:
                        car.speed = 0
                    # Map edge check
                    bounds = network.bounds
                    if car.x < 0 or car.x > bounds[2] \
                            or car.y < 0 or car.y > bounds[3]:
                        car.speed = 0
                        car.x = max(0, min(bounds[2], car.x))
                        car.y = max(0, min(bounds[3], car.y))
        _t_b = time.perf_counter()
        _t_c = _t_b   # validate+guard now run inside the physics phase

        # --- Render interpolation (fix-your-timestep) ----
        # Physics advanced in fixed dt_fixed substeps above; a rendered
        # frame can contain 0 or 2 of them. Draw the car at lerp(prev,
        # curr, alpha) so its on-screen motion is smooth every frame
        # instead of freezing/hopping when the step count quantises.
        if car is None or frozen:
            if car is not None:
                car._render_state = None
            _rx, _ry = camera.x, camera.y   # no car / paused: hold the view
        elif _prev_render is not None:
            px, py, ph = _prev_render
            cx, cy, ch = car.x, car.y, car.heading
            if (cx - px) ** 2 + (cy - py) ** 2 > 100.0:
                # Teleport / snap: no lerping across the jump.
                car._render_state = (cx, cy, ch)
            else:
                alpha = _physics_accum / dt_fixed
                dh = (ch - ph + 540.0) % 360.0 - 180.0
                car._render_state = (
                    px + (cx - px) * alpha,
                    py + (cy - py) * alpha,
                    (ph + dh * alpha) % 360.0,
                )
            _rx, _ry = car._render_state[:2]
        else:
            _rx, _ry = car.x, car.y

        # Resolve the pending RED end flag once the current route covers
        # that segment (it may lie beyond the initial route horizon):
        # position + travel heading come from the route's node order, so
        # "right of the road" is correct even for backward-traversed
        # segments.
        if renderer.flag_red_pending and car is not None \
                and car.bicycle_nav is not None:
            _fseg, _fprog = renderer.flag_red_pending
            if _fseg in getattr(car.bicycle_nav, '_route_seg_set', set()):
                _pos = _flag_position_on_route(network, car.bicycle_nav,
                                               _fseg, _fprog)
                if _pos:
                    _fx, _fy, _fh = _pos
                    # The RED flag is the car's DESTINATION unless this is a
                    # running turn test (flag_red_nav=False): then it is a
                    # visual end-of-test marker only - the car drives THROUGH
                    # it and the harness ends the test on the crossing.
                    if renderer.flag_red_nav:
                        # Truncate the reference line at the centreline
                        # point so the car parks AT the flag (parking ramp +
                        # kerb drift + stop), not at whatever dead end the
                        # route happens to reach beyond it.
                        print(f"[FLAGDBG] dest set seg={_fseg} prog={_fprog} "
                              f"px=({_fx:.0f},{_fy:.0f})")
                        car.bicycle_nav.set_destination(_fx, _fy)
                    else:
                        print(f"[FLAGDBG] visual-only flag seg={_fseg} "
                              f"prog={_fprog} px=({_fx:.0f},{_fy:.0f})")
                    # The route point sits on the segment CENTRELINE; shift
                    # it onto the right kerb (the same offset the car uses)
                    # so _draw_flag's 3 m grass offset lands the pennant
                    # fully off the carriageway - like the green flag, which
                    # is placed from the car's own kerb-side position.
                    _frad = math.radians(_fh)
                    _fright = (math.cos(_frad), -math.sin(_frad))
                    _foff = kerb_offset_m(network.segments[_fseg].width)
                    renderer.flag_red = [
                        _fx + _fright[0] * _foff,
                        _fy + _fright[1] * _foff,
                        _fh,
                    ]
                    renderer.flag_red_pending = None

        _t_d = time.perf_counter()
        # Camera follow (only when moving and not frozen) - follows the
        # INTERPOLATED position so road and car stay in sync on screen.
        if not frozen:
            camera.update(_rx, _ry, network.world_width, network.world_height,
                          follow=(car is not None and abs(car.speed) > 0.1))

        # Render
        screen.fill(BG_COLOR)  # solid grass-green background
        renderer.draw(screen, car)
        obstacle_ui.draw_world(screen)   # obstacles: above roads, below car/HUD
        if car is not None:
            car.draw(screen, camera)
            renderer.draw_trail(screen, car)  # after sprite so buckets are visible
        
        # HUD: off-road warning LED (FREE mode, solid red with "OFF")
        if free_off_road:
            wx = WINDOW_WIDTH - 50
            wy = WINDOW_HEIGHT - 90
            pygame.draw.circle(surface=screen, color=(120, 0, 0), center=(wx, wy), radius=16)
            pygame.draw.circle(surface=screen, color=(255, 30, 30), center=(wx, wy), radius=16, width=3)
            screen.blit(renderer._hud_text("off_road", "OFF-ROAD", 12, (255, 80, 80)),
                        (wx - 24, wy - 24))

        # FROZEN overlay: while paused the driver logic (blinkers, turns)
        # is skipped entirely, so a frozen game looks dead. Make it
        # unmistakable on screen - previously the only hint was a console
        # print, and players thought the controls were broken.
        if frozen:
            overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT),
                                     pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 110))
            screen.blit(overlay, (0, 0))
            t1 = renderer._hud_text("frozen_big", "FROZEN", 48, (255, 255, 255))
            t2 = renderer._hud_text("frozen_small", "press ESC to resume",
                                    20, (255, 220, 120))
            screen.blit(t1, ((WINDOW_WIDTH - t1.get_width()) // 2,
                             WINDOW_HEIGHT // 2 - 40))
            screen.blit(t2, ((WINDOW_WIDTH - t2.get_width()) // 2,
                             WINDOW_HEIGHT // 2 + 20))

        # Obstacle palette panel + drag ghost (topmost layer, next to the
        # minimap; must stay visible above the frozen overlay so layouts can
        # be edited while the simulation is paused).
        obstacle_ui.draw_panel(screen)

        # Quit button (top-left, topmost layer): click for an orderly
        # shutdown - see _quit_rect above.
        pygame.draw.rect(screen, (178, 58, 58), _quit_rect, border_radius=6)
        pygame.draw.rect(screen, (235, 235, 235), _quit_rect,
                         width=1, border_radius=6)
        _qt = renderer._hud_text("quit_btn", "Quit", 18, (255, 255, 255))
        screen.blit(_qt, (_quit_rect.centerx - _qt.get_width() // 2,
                          _quit_rect.centery - _qt.get_height() // 2))

        # HUD: wrong-side warning LED (blinks red when on opposing lane).
        # In FREE mode this is the ONLY consequence - the process survives.
        if on_wrong_side:
            # Blink at ~3 Hz (on for 10 frames, off for 10 frames at 60 fps)
            if frame % 20 < 15:
                wx = WINDOW_WIDTH - 50
                wy = WINDOW_HEIGHT - 50
                pygame.draw.circle(surface=screen, color=(120, 0, 0), center=(wx, wy), radius=16)
                pygame.draw.circle(surface=screen, color=(255, 30, 30), center=(wx, wy), radius=16, width=3)
                screen.blit(renderer._hud_text("wrong_side", "WRONG SIDE", 12, (255, 80, 80)),
                            (wx - 40, wy - 24))

        pygame.display.flip()
        _t_e = time.perf_counter()

        # TEMP perf probe (remove after diagnosis)
        if not smoke_test_frames and frame % 120 == 0:
            print(f"[PERF] f={frame} tick={frame_ms:.0f}ms "
                  f"update={(_t_b - _t_a) * 1000:.1f} validate={(_t_c - _t_b) * 1000:.1f} "
                  f"guard={(_t_d - _t_c) * 1000:.1f} render={(_t_e - _t_d) * 1000:.1f}",
                  flush=True)
        # TEMP judder probe: physics substeps per frame. A pattern with
        # many 0-step or 2-step frames means the fixed-timestep accumulator
        # is aliasing against the render rate (visible as motion judder).
        if not smoke_test_frames:
            main._step_log = getattr(main, '_step_log', [])
            main._step_log.append(_steps_this_frame)
            if len(main._step_log) >= 600:
                _sl = main._step_log
                main._step_log = []
                _c0 = _sl.count(0); _c1 = _sl.count(1); _c2 = sum(1 for s in _sl if s > 1)
                print(f"[STEPS] f={frame} frames=600 "
                      f"0-step={_c0} 1-step={_c1} >1-step={_c2} "
                      f"pattern={''.join(str(min(s, 2)) for s in _sl[:48])}",
                      flush=True)

        # On freeze: capture this frame and copy to clipboard
        if esc_pressed_now and frozen:
            copy_screenshot_to_clipboard(screen)
        
        # Update API state and screenshot (if API enabled)
        if api:
            if car is None:
                # No car on the map yet (test maps don't auto-spawn):
                # report that, keep camera info current.
                api.update_state({
                    'frame': frame,
                    'time': frame * dt_fixed if smoke_test_frames else frame / 60.0,
                    'has_car': False,
                    'frozen': frozen,
                    'validator_enabled': validator.enabled,
                    'validator_violations': len(validator.violations),
                    'camera_x': camera.x,
                    'camera_y': camera.y,
                    'camera_zoom': camera.zoom,
                })
            else:
                # Parking state lives on the nav, not the driver (reading
                # car.driver yields None and silently disables the e2e
                # suite's reverse-in gate).
                _nav = getattr(car, 'bicycle_nav', None)
                api.update_state({
                    'frame': frame,
                    'time': frame * dt_fixed if smoke_test_frames else frame / 60.0,
                    'has_car': True,
                    # Monotonic per-car identity: the teleport ack. has_car
                    # alone can't confirm a NEW car - it is already True
                    # while the old one still exists (see the runner's
                    # _wait_for_new_car).
                    'car_uid': car.uid,
                    'x': car.x,
                    'y': car.y,
                    'heading': car.heading,
                    'speed': car.speed,
                    'speed_kmh': car.speed * 3.6,
                    'segment': car.seg_idx,
                    'progress': car.progress,
                    'segment_length': network.segments[car.seg_idx].length,
                    'forward': car.forward,
                    'distance_to_junction': _distance_to_junction(car, network),
                    'on_road': car.is_on_road(network),
                    # Dashboard lamps for the cockpit controller (tools/controller.py)
                    'braking': bool(getattr(car, '_braking', False)),
                    'accelerating': bool(getattr(car, '_accelerating', False)),
                    'wrong_side': on_wrong_side,
                    'driver': car.driver.get_name(),
                    # Parking state for the e2e suite: a reverse-in park
                    # deliberately crosses the flag to stage the back-in, so
                    # the suite must wait for 'parked' instead of latching
                    # arrival at the flag (see tests/test_turning.py).
                    'parking': {
                        'style': getattr(_nav, '_park_style', None),
                        'phase': getattr(_nav, 'park_phase', 'none'),
                        'parked': bool(getattr(_nav, '_parked', False)),
                        'reversing':
                            getattr(_nav, '_reverse_park', None) is not None,
                    },
                    'trail_enabled': car.trail_enabled,
                    'validator_enabled': validator.enabled,
                    'validator_violations': len(validator.violations),
                    'hazard': bool(getattr(car.driver, 'hazard', False)),
                    'hazard_reason': getattr(car.driver, 'hazard_reason', ''),
                    'frozen': frozen,
                    'blinker_left': bool(getattr(car.driver, 'blinker_left', False)),
                    'blinker_right': bool(getattr(car.driver, 'blinker_right', False)),
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
        if car is not None:
            print(f"Smoke test OK: {frame} frames, car at ({car.x:.0f}, {car.y:.0f}), "
                  f"speed={car.speed:.1f} m/s, zoom={camera.zoom:.2f}")
        else:
            print(f"Smoke test OK: {frame} frames, no car on map, "
                  f"zoom={camera.zoom:.2f}")
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
