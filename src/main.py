#!/usr/bin/env python3
"""Car Game — Entry Point (headless world simulator, M5)

The sim runs without any window: physics + AI driver + REST API. The
Godot frontend (driving-game repo) is the renderer - it polls /state
and mirrors the sim camera. There is no pygame anywhere in this package.
"""

import math
import os
import sys
import time

from .config import *
from .osm_loader import fetch_osm_data
from .road_network import RoadNetwork
from .camera import Camera
from .car import Car
from .driver import Driver, KeyboardDriver, BicycleDriver
from .physics_validator import PhysicsValidator
from .lane_guard import LaneGuard
from .obstacles import ObstacleManager
from .rest_api import GameAPI
from .test_maps import build_test_map, TEST_MAPS


class TestFlags:
    """Server-side state of the test pennants + HUD label.

    Lives here (not in a renderer) since M5: the remote frontend draws
    them from /state, the sim only owns their positions. flag_red is
    given as [segment, progress] and resolved to a map position by the
    main loop once the car's route covers that segment.
    """
    def __init__(self):
        self.flag_green: list | None = None
        self.flag_red: list | None = None
        self.flag_red_pending: tuple | None = None
        self.flag_red_nav: bool = True
        self.hud_label: str | None = None


class _NoKeys:
    """Headless key state: nothing is pressed. API control drives the car."""
    def __getitem__(self, key):
        return False


_NO_KEYS = _NoKeys()


def _distance_to_junction(car, network) -> float:
    """Metres from the car to the junction it is heading toward (for the API)."""
    seg = network.segments[car.seg_idx]
    return ((1.0 - car.progress) if car.forward else car.progress) * seg.length


def _flags_payload(tf):
    """/state export of one car's pennants (world px + heading, or None)."""
    return {
        'green': list(tf.flag_green) if tf and tf.flag_green else None,
        'red': list(tf.flag_red) if tf and tf.flag_red else None,
    }


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
    """Run the simulator (headless). If smoke_test_frames > 0, run for that
    many frames and exit - used by CI to prove the sim stays alive."""
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
        # Multi-car color (docs/MULTI_CAR_PLAN.md): deterministic per uid,
        # red first, then the old pygame obstacle palette colors.
        car.color = CAR_COLORS[(car.uid - 1) % len(CAR_COLORS)]
        return car

    def _create_car_at_segment(seg_idx: int, progress: float = 0.5):
        """Create a car at `progress` (0..1) along segment `seg_idx`,
        driving start_node -> end_node in the normal driving position.

        Chord-based placement: exact on smooth tracks where the chords hug
        the rounded path (the figure-8's 48 short segments), but do NOT use
        it for sharp-corner segments - see _spawn_position's note on why
        chord spawns sit off-pavement near hairpins. Used by POST /teleport
        {"segment": N} (stress tests, dev)."""
        seg = network.segments[seg_idx]
        x = seg.x1 + (seg.x2 - seg.x1) * progress
        y = seg.y1 + (seg.y2 - seg.y1) * progress
        h = math.degrees(math.atan2(seg.x2 - seg.x1, seg.y2 - seg.y1)) % 360.0
        # Same right-lane-centre offset _spawn_position uses for named start
        # points; the nav holds this line (lane_offset_override_m).
        offset_m = lane_base_offset_m(seg.width, seg.lanes,
                                      seg.parking_lane_width, seg.oneway)
        rad = math.radians(h)
        x += math.cos(rad) * offset_m * PIXELS_PER_METER
        y -= math.sin(rad) * offset_m * PIXELS_PER_METER
        car = Car(x, y, h, seg_idx, BicycleDriver())
        car.progress = progress
        car.forward = True
        car.lane_offset_override_m = offset_m
        # Breadcrumbs on like every other spawn.
        car.trail_enabled = True
        car.color = CAR_COLORS[(car.uid - 1) % len(CAR_COLORS)]
        return car

    # --- Car(s) with AI driver (multi-car, docs/MULTI_CAR_PLAN.md) ---
    # All live cars in one dict; _follow_uid is the PRIMARY car - the sim
    # camera follows it and the legacy top-level /state fields describe it,
    # so single-car consumers (e2e suite, cockpit) keep working unchanged.
    # Synthetic test maps do NOT auto-spawn a car: the e2e suite teleports
    # its own car in (POST /teleport), and an idle AI car driving around
    # before the first teleport is noise that makes every run less
    # reproducible. On a test map, --start <name> focuses the CAMERA on
    # that start point (the suite's car then appears exactly there); real
    # OSM data keeps its random spawn.
    cars: dict[int, Car] = {}                 # uid -> car
    car_flags: dict[int | None, TestFlags] = {}   # uid -> pennants/label;
                                                 # key None = unaddressed
                                                 # (legacy clients without a
                                                 # uid; adopted by the next
                                                 # spawned car)
    _follow_uid = None

    def _remove_car(uid):
        """Remove one car + its flags; re-point the follow at the newest
        remaining car if the followed one goes."""
        nonlocal _follow_uid
        cars.pop(uid, None)
        car_flags.pop(uid, None)
        if _follow_uid == uid:
            _follow_uid = max(cars) if cars else None

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
        # Real OSM data: random spawn as before.
        car = _create_car(start_name)
        cars[car.uid] = car
        car_flags[car.uid] = TestFlags()
        _follow_uid = car.uid
        if start_name:
            print(f"Spawn: '{start_name}' (deterministic; --start <name> to change)")
    print("Navigation model: BICYCLE (kinematic, free particle)")
    
    # --- Physics validator (toggle via POST /toggle {"validator": false}) ---
    validator = PhysicsValidator(enabled=True)
    print("Physics validator: ENABLED")
    
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
        # Publish named start points (synthetic test maps only; empty for
        # OSM data). Kept OUT of /state: static map data belongs on its own
        # endpoint, and merging it into game_state made /state serve just
        # {"start_points": ...} during the startup window (Flask up, first
        # frame not yet published) - see GameAPI.start_points.
        api.set_start_points({
            name: {'x': x, 'y': y, 'heading': h, 'segment': seg,
                   'forward': fwd, 'lateral_offset_m': off}
            for name, (x, y, h, seg, fwd, off) in network.start_points.items()
        })
    elif not smoke_test_frames:
        print("\n💡 Tip: Run with --api to enable REST API for remote control")
        print("   Example: python -m src.main --api")
        print("   Then: curl http://localhost:5000/state\n")

    # --- Sim camera (mirrored by the remote renderer via /state) ---
    camera = Camera(WINDOW_WIDTH, WINDOW_HEIGHT)
    if car is not None:
        camera.x, camera.y = car.x, car.y   # snap to car at start
    if map_name:
        # Test maps open at a close driving zoom (7x), focused on the
        # --start point when given, else on the world centre - NOT fitted
        # to show the whole map: you want to watch the maneuver up close.
        # The suite's teleport then places the car exactly where the view
        # is looking; the Godot client mirrors this camera from /state.
        camera.zoom = 7.0
        if focus_point is not None:
            camera.x, camera.y = focus_point
        else:
            camera.x, camera.y = network.world_width / 2, network.world_height / 2

    # --- Game loop (headless, paced at 60 Hz) ---
    frame = 0
    running = True
    frozen = False
    dt_fixed = 1 / 60
    _last_frame_wall = time.perf_counter()
    _physics_accum = 0.0
    # SIM CLOCK = total physics substeps executed (NOT the frame counter).
    # A slow wall-clock frame runs 2 substeps in one iteration; if the
    # exported clock only advanced per iteration, position and time would
    # decouple (the car moves 2x while time advances 1x -> every client
    # interpolating on sim time sees a periodic forward jump). Tying the
    # clock to the substeps keeps them consistent by construction.
    _sim_steps_total = 0
    _prev_render = {}     # uid -> (x, y, heading) before the last substep
    while running:
        frame += 1
        if smoke_test_frames and frame > smoke_test_frames:
            running = False
            break

        # Pacing: measure the real elapsed time since the last frame and
        # sleep out the remainder of this frame's 60 Hz budget (the
        # headless replacement for clock.tick(60)). A slow frame simply
        # sleeps less; the fixed-timestep accumulator below catches up
        # with capped substeps, so a stall never becomes a leap.
        # Pacing (headless replacement for clock.tick(60)): measure the
        # whole previous cycle (work + sleep) at the top, do the work,
        # then sleep out the remainder of THIS frame's 60 Hz budget at the
        # bottom. Anchoring the sleep target one cycle early instead would
        # halve the steady-state period (~107 fps).
        _now = time.perf_counter()
        if not smoke_test_frames:
            frame_ms = (_now - _last_frame_wall) * 1000.0
            _last_frame_wall = _now
        else:
            frame_ms = 1000.0 / 60
        dt = frame_ms / 1000.0
        # Clamp the physics step. If a frame stalls (route rebuild, GC),
        # the real elapsed time can be hundreds of milliseconds; integrating
        # that in one go teleports the car metres downroad, straight through
        # any geometry in between. Better to run briefly in slow motion than
        # to take an unphysical leap.
        dt = min(dt, 1.0 / 30.0)

        # Key state: headless - nothing is pressed; the REST API drives the
        # car (POST /control). The smoke test simulates inputs directly.
        if smoke_test_frames:
            keys = {KEY_UP: True,
                    KEY_RIGHT: frame > smoke_test_frames // 2}
        else:
            keys = _NO_KEYS
        
        # Handle API commands (if API enabled). Commands arrive as QUEUES
        # (key -> [payload, ...]) so parallel clients can't overwrite each
        # other between frames; payloads apply in arrival order. Any bad
        # payload is logged and skipped - one broken client must never
        # kill the whole simulation (multi-car test runs).
        if api:
            commands = api.get_commands()
            # [(key, payload), ...] in GLOBAL arrival order - a
            # 'clear then spawn' burst must apply the clear FIRST
            # even when both land within one frame batch.
            for _ckey, _cpayload in commands:
                try:
                    # Teleport command: default REPLACES the current car(s)
                    # (legacy behavior - e2e suite, cockpit); "add": true ADDS
                    # a fresh car alongside them (parallel test runs,
                    # docs/MULTI_CAR_PLAN.md).
                    if _ckey == 'teleport':
                        tp = _cpayload or {}
                        if 'segment' in tp:
                            # Arbitrary-segment spawn (stress tests, dev):
                            # chord placement along the segment.
                            new_car = _create_car_at_segment(
                                int(tp['segment']),
                                float(tp.get('progress', 0.5)))
                        else:
                            start_point = tp.get('start_point') or None
                            if not tp.get('add'):
                                for _uid in list(cars):
                                    _remove_car(_uid)
                            new_car = _create_car(start_point,
                                                  progress=tp.get('progress'))
                        # Optional rolling start (m/s): the running turn tests
                        # spawn already moving instead of accelerating from a
                        # standstill.
                        if tp.get('speed') is not None:
                            new_car.speed = float(tp['speed'])
                        cars[new_car.uid] = new_car
                        car_flags[new_car.uid] = TestFlags()
                        # Unaddressed flags/label (legacy clients that set them
                        # before any car existed) are adopted by the new car.
                        if None in car_flags:
                            car_flags[new_car.uid] = car_flags.pop(None)
                        _follow_uid = new_car.uid
                        camera.snap_to(new_car.x, new_car.y,
                                       network.world_width, network.world_height)
                        print(f"\n🔄 {'Added' if tp.get('add') else 'New'} car "
                              f"#{new_car.uid} ({new_car.color}) at segment "
                              f"{new_car.seg_idx}, heading {new_car.heading:.1f}°"
                              f"{' (rolling start)' if tp.get('speed') is not None else ''}\n")

                    # Multi-car cleanup (POST /cars): clear all / remove one.
                    elif _ckey == 'cars':
                        cc = _cpayload or {}
                        action = cc.get('action')
                        if action == 'clear':
                            for _uid in list(cars):
                                _remove_car(_uid)
                            print("API: all cars removed")
                        elif action == 'remove' and cc.get('uid') is not None:
                            _remove_car(int(cc['uid']))

                    # Toggle command (per-car part resolves an optional uid;
                    # omit = primary car; "validator" stays global)
                    elif _ckey == 'toggle':
                        toggle_params = _cpayload or {}
                        _tuid = toggle_params.get('uid') or _follow_uid
                        if _tuid not in cars:
                            _tuid = _follow_uid
                        tcar = cars.get(_tuid) if _tuid is not None else None
                        if 'breadcrumbs' in toggle_params and tcar is not None:
                            tcar.trail_enabled = toggle_params['breadcrumbs']
                            print(f"API: Breadcrumbs "
                                  f"{'ON' if tcar.trail_enabled else 'OFF'} (car #{_tuid})")
                        if 'validator' in toggle_params:
                            if toggle_params['validator']:
                                validator.enable()
                            else:
                                validator.disable()
                        if 'mode' in toggle_params and tcar is not None:
                            mode = toggle_params['mode']
                            if mode == 'bicycle' and not isinstance(tcar.driver, BicycleDriver):
                                tcar.driver = BicycleDriver()
                                tcar.bicycle_nav = None
                                print(f"API: Switched to BICYCLE mode (car #{_tuid})")
                            elif mode == 'free' and not isinstance(tcar.driver, KeyboardDriver):
                                tcar.driver = KeyboardDriver()
                                print(f"API: Switched to FREE mode (car #{_tuid})")

                    # Label command (short per-car HUD text, e.g. "2/3").
                    elif _ckey == 'label':
                        ld = _cpayload or {}
                        _luid = ld.get('uid') or _follow_uid
                        if _luid is None or _luid not in cars:
                            _luid = None     # unaddressed: adopted at next spawn
                        car_flags.setdefault(_luid, TestFlags()).hud_label = \
                            ld.get('text')

                    # Freeze / resume (replaces the old ESC key): several
                    # queued freezes resolve to the LAST one.
                    elif _ckey == 'freeze':
                        frozen = bool((_cpayload or {}).get('frozen'))
                        print("\n⏸️  API: simulation FROZEN" if frozen
                              else "▶️  API: simulation resumed")

                    # Test confirmation flags (PER CAR): green at the scenario
                    # start (car position, immediate), red at its end given as
                    # [segment, progress] - resolved to a map position by the
                    # main loop once THAT car's route covers that segment, so
                    # it is visible from the START of the test. Optional "uid"
                    # addresses a specific car; omit = primary (legacy clients).
                    elif _ckey == 'flags':
                        fl = _cpayload or {}
                        if not isinstance(fl, dict):
                            continue
                        _fuid = fl.get('uid') or _follow_uid
                        if _fuid is None or _fuid not in cars:
                            _fuid = None     # unaddressed: adopted at next spawn
                        tf = car_flags.setdefault(_fuid, TestFlags())
                        fcar = cars.get(_fuid) if _fuid is not None else None
                        # Only update the keys that are present, so setting the
                        # end flag doesn't wipe the start flag (and vice versa).
                        if 'green' in fl:
                            tf.flag_green = fl['green']
                        if 'red' in fl:
                            if fl['red'] is None:
                                tf.flag_red = None
                                tf.flag_red_pending = None
                                if fcar is not None and fcar.bicycle_nav is not None:
                                    fcar.bicycle_nav.clear_destination()
                            else:
                                seg_idx, prog = fl['red'][0], fl['red'][1]
                                tf.flag_red = None
                                tf.flag_red_pending = (seg_idx, prog)
                                # red_nav (default True): the flag becomes the
                                # car's navigation destination (park at it).
                                # Running turn tests send red_nav=False - the
                                # flag is a visual end-of-test marker only
                                # (docs/TESTING.md).
                                tf.flag_red_nav = bool(fl.get('red_nav', True))

                    # Hazard lights (Warnblinkanlage): explicit on/off command,
                    # per-car via uid (omit = primary).
                    elif _ckey == 'hazard':
                        hz = _cpayload or {}
                        _huid = hz.get('uid') or _follow_uid
                        hcar = cars.get(_huid) if _huid is not None else None
                        if hcar is not None and isinstance(hcar.driver, BicycleDriver):
                            hcar.driver.set_hazard(
                                bool(hz.get('on')),
                                reason="manual (REST API)" if hz.get('on')
                                       else "manual off (REST API)")
                except Exception as e:
                    print(f"⚠️  API command error (skipped, sim continues): {e!r}",
                          flush=True)

        # --- Per-car control input (driver + API merge), once per frame;
        # the fixed-timestep substeps below reuse it (as before). ---
        car_control = {}
        if not frozen and cars:
            for c in cars.values():
                # Get control input from driver (keyboard or API)
                ci = c.driver.get_control(c, network, dt, keys)
                if api:
                    is_primary = (c.uid == _follow_uid)
                    api_control = api.get_control_for(c.uid,
                                                      is_primary=is_primary)
                    # API control overrides keyboard for specific keys
                    if api_control['accelerate']:
                        ci['accelerate'] = True
                    if api_control['brake']:
                        ci['brake'] = True
                    if api_control['steer_left']:
                        ci['steer_left'] = True
                    if api_control['steer_right']:
                        ci['steer_right'] = True
                    if api_control['blinker_left']:
                        ci['blinker_left'] = True
                    if api_control['blinker_right']:
                        ci['blinker_right'] = True

                    # BicycleDriver's turn choice comes from `pending_turn`.
                    # API blinkers are ONE-SHOT commands (like flicking a
                    # real indicator): apply them once, then consume the
                    # flag so they don't re-trigger every frame. The
                    # driver's own blinker state persists afterwards and is
                    # cleared by the car itself via the mechanical auto-off
                    # (steered in + steered back = off).
                    if hasattr(c.driver, 'signal_turn'):
                        if api_control['blinker_left']:
                            c.driver.signal_turn('left')
                            api.clear_control('blinker_left', c.uid,
                                              is_primary=is_primary)
                        elif api_control['blinker_right']:
                            c.driver.signal_turn('right')
                            api.clear_control('blinker_right', c.uid,
                                              is_primary=is_primary)
                    # U-turn (Wenden) is a one-shot command, like a blinker.
                    if api_control.get('uturn') \
                            and hasattr(c.driver, 'uturn_requested'):
                        c.driver.uturn_requested = True
                        api.clear_control('uturn', c.uid,
                                          is_primary=is_primary)
                car_control[c.uid] = ci
        
        # --- Physics: FIXED-timestep substeps (only when not frozen) ----
        # Integrate at exactly dt_fixed (1/60 s) no matter how long the
        # frame took. Variable dt let render hitches (tile generation,
        # window drags, GC) change the integration steps and shifted
        # cornering trajectories by more than the 0.35 m kerb clearance -
        # the same scenario passed headless but clipped off-road live.
        # Fixed steps make the live game deterministic: if a scenario is
        # clean in a headless run, it is clean here.
        _t_a = time.perf_counter()
        per_car_state = {}   # uid -> frame outputs (wrong side etc.) for /state
        if frozen or not cars:
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
                _sim_steps_total += 1
                # ALL cars step together in the same substep (shared
                # accumulator): every car advances exactly dt_fixed per
                # substep, so a multi-car run is deterministic like the
                # single-car one.
                for c in cars.values():
                    _prev_render[c.uid] = (c.x, c.y, c.heading)
                    _pre_x, _pre_y, _pre_h = c.x, c.y, c.heading
                    c.update(dt_fixed, network, car_control.get(c.uid, {}))
                    # (Blinker auto-cancel is mechanical - the driver
                    # watches the steering angle itself: steered in +
                    # steered back = off. No segment-change hook needed.)
                    # Stop-on-contact with obstacles (ALL modes;
                    # docs/OBSTACLES.md): brake at full A_BRAKE and clamp
                    # so the body box never interpenetrates an obstacle -
                    # the car rests against it.
                    in_contact = obstacle_mgr.apply_contact_stop(
                        c, dt_fixed, _pre_x, _pre_y, _pre_h)
                    # Physics validation (independent check). While in
                    # contact the motion is externally constrained by a
                    # solid object, so the validator suspends the
                    # turning-radius invariant for that frame -
                    # jump/snap/off-road checks keep running.
                    validator.check(c, dt_fixed, network,
                                    in_contact=in_contact)
                    # Lane guard: wrong-side driving (skipped during active
                    # turns - lateral offset from the incoming centreline is
                    # expected; and for the whole U-turn, where crossing the
                    # centreline is INTENDED, spec §5).
                    in_turn = hasattr(c.bicycle_nav, '_s') \
                        and c.bicycle_nav._s is not None \
                        and c.bicycle_nav._in_turn_blend_zone(c.bicycle_nav._s)
                    uturn_now = c.bicycle_nav is not None and \
                                getattr(c.bicycle_nav, 'uturn_active', False)
                    on_wrong_side = False
                    if not in_turn and not uturn_now:
                        # Warning only - NEVER fatal here (no crash, no
                        # freeze): the guard prints once per crossing and
                        # /state reports 'wrong_side' + cumulative
                        # lane-guard stats; FAILING a run is the e2e suite's
                        # job (docs/TESTING.md §3). In FREE mode the same
                        # flag drives the red LED + label.
                        on_wrong_side = lane_guard.check(c, dt_fixed, network)
                    # Off-road check (FREE mode only): stop the car + warning
                    free_off_road = False
                    if c.driver.get_name() == "FREE":
                        free_off_road = not c.is_on_road(network)
                        if free_off_road:
                            c.speed = 0
                        # Map edge check
                        bounds = network.bounds
                        if c.x < 0 or c.x > bounds[2] \
                                or c.y < 0 or c.y > bounds[3]:
                            c.speed = 0
                            c.x = max(0, min(bounds[2], c.x))
                            c.y = max(0, min(bounds[3], c.y))
                    per_car_state[c.uid] = {
                        'on_wrong_side': on_wrong_side,
                        'free_off_road': free_off_road,
                    }
        _t_b = time.perf_counter()
        _t_c = _t_b   # validate+guard now run inside the physics phase

        # --- Camera follow position (fix-your-timestep interpolation) ----
        # Physics advanced in fixed dt_fixed substeps above; a rendered
        # frame can contain 0 or 2 of them. Follow the FOLLOWED car's
        # interpolated position so the mirrored camera stays smooth every
        # frame instead of freezing/hopping when the step count quantises.
        followed = cars.get(_follow_uid) if _follow_uid is not None else None
        if frozen or followed is None:
            _rx, _ry = camera.x, camera.y   # no car / paused: hold the view
        elif followed.uid in _prev_render:
            px, py, ph = _prev_render[followed.uid]
            cx, cy, ch = followed.x, followed.y, followed.heading
            if (cx - px) ** 2 + (cy - py) ** 2 > 100.0:
                # Teleport / snap: no lerping across the jump.
                _rx, _ry = cx, cy
            else:
                alpha = _physics_accum / dt_fixed
                _rx = px + (cx - px) * alpha
                _ry = py + (cy - py) * alpha
        else:
            _rx, _ry = followed.x, followed.y

        # Resolve each car's pending RED end flag once ITS route covers that
        # segment (it may lie beyond the initial route horizon): position +
        # travel heading come from the route's node order, so "right of the
        # road" is correct even for backward-traversed segments.
        for c in list(cars.values()):
            tf = car_flags.get(c.uid)
            if tf is None or not tf.flag_red_pending \
                    or c.bicycle_nav is None:
                continue
            _fseg, _fprog = tf.flag_red_pending
            if _fseg in getattr(c.bicycle_nav, '_route_seg_set', set()):
                _pos = _flag_position_on_route(network, c.bicycle_nav,
                                               _fseg, _fprog)
                if _pos:
                    _fx, _fy, _fh = _pos
                    # The RED flag is the car's DESTINATION unless this is a
                    # running turn test (flag_red_nav=False): then it is a
                    # visual end-of-test marker only - the car drives THROUGH
                    # it and the harness ends the test on the crossing.
                    if tf.flag_red_nav:
                        # Truncate the reference line at the centreline
                        # point so the car parks AT the flag (parking ramp +
                        # kerb drift + stop), not at whatever dead end the
                        # route happens to reach beyond it.
                        print(f"[FLAGDBG] car #{c.uid} dest set seg={_fseg} "
                              f"prog={_fprog} px=({_fx:.0f},{_fy:.0f})")
                        c.bicycle_nav.set_destination(_fx, _fy)
                    else:
                        print(f"[FLAGDBG] car #{c.uid} visual-only flag "
                              f"seg={_fseg} prog={_fprog} "
                              f"px=({_fx:.0f},{_fy:.0f})")
                    # The route point sits on the segment CENTRELINE; shift
                    # it onto the right kerb (the same offset the car uses)
                    # so the renderer's 3 m grass offset lands the pennant
                    # fully off the carriageway - like the green flag, which
                    # is placed from the car's own kerb-side position.
                    _frad = math.radians(_fh)
                    _fright = (math.cos(_frad), -math.sin(_frad))
                    _foff = kerb_offset_m(network.segments[_fseg].width)
                    tf.flag_red = [
                        _fx + _fright[0] * _foff,
                        _fy + _fright[1] * _foff,
                        _fh,
                    ]
                    tf.flag_red_pending = None

        _t_d = time.perf_counter()
        # Camera follow (only when moving and not frozen) - follows the
        # INTERPOLATED position; the remote renderer mirrors this camera.
        if not frozen:
            camera.update(_rx, _ry, network.world_width, network.world_height,
                          follow=(followed is not None
                                  and abs(followed.speed) > 0.1))

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

        # Sleep out the remainder of this frame's budget (see pacing note
        # at the top of the loop). Plain time.sleep overshoots badly on
        # this platform (a 16.7 ms sleep took 25 ms - timer quantization),
        # which dragged the sim down to ~43 fps; so sleep only most of the
        # remainder and busy-spin the last slice for exact pacing.
        if not smoke_test_frames:
            _rem = dt_fixed - (time.perf_counter() - _now)
            if _rem > 0:
                _SPIN_S = 0.012   # spin window: absorbs the sleep overshoot
                if _rem > _SPIN_S:
                    time.sleep(_rem - _SPIN_S)
                while time.perf_counter() < _now + dt_fixed:
                    pass

        # Update API state (if API enabled)
        if api:
            # Multi-car (docs/MULTI_CAR_PLAN.md): the top-level fields keep
            # describing the PRIMARY (followed) car - legacy consumers (e2e
            # suite, cockpit) are untouched - while `cars` carries ALL cars;
            # the Godot frontend prefers that array.
            _tf_p = car_flags.get(followed.uid) if followed is not None \
                else None
            cars_payload = []
            for c in cars.values():
                _pcs = per_car_state.get(c.uid, {})
                cars_payload.append({
                    'car_uid': c.uid,
                    'x': c.x,
                    'y': c.y,
                    'heading': c.heading,
                    'speed_kmh': c.speed * 3.6,
                    'level': network.segments[c.seg_idx].level,
                    'segment': c.seg_idx,
                    'progress': c.progress,
                    'on_road': c.is_on_road(network),
                    'wrong_side': _pcs.get('on_wrong_side', False),
                    'blinker_left': bool(
                        getattr(c.driver, 'blinker_left', False)),
                    'blinker_right': bool(
                        getattr(c.driver, 'blinker_right', False)),
                    'hazard': bool(getattr(c.driver, 'hazard', False)),
                    'color': c.color,
                    'flags': _flags_payload(car_flags.get(c.uid)),
                    'hud_label': car_flags[c.uid].hud_label
                        if c.uid in car_flags else None,
                })
            if followed is None:
                # No car on the map yet (test maps don't auto-spawn):
                # report that, keep camera info current.
                api.update_state({
                    'frame': frame,
                    'time': _sim_steps_total * dt_fixed,
                    'has_car': False,
                    'frozen': frozen,
                    'validator_enabled': validator.enabled,
                    # Per-car counters: no car -> nothing logged yet.
                    'validator_violations': 0,
                    # Test flags (green start / red end pennant) in world
                    # pixels, or None: the Godot renderer draws them.
                    'flags': {'green': None, 'red': None},
                    # Short HUD label (e.g. "5/21") set via POST /label -
                    # the remote renderer shows it like pygame did.
                    'hud_label': None,
                    'camera_x': camera.x,
                    'camera_y': camera.y,
                    'camera_zoom': camera.zoom,
                    'cars': cars_payload,
                })
            else:
                car = followed
                # Parking state lives on the nav, not the driver (reading
                # car.driver yields None and silently disables the e2e
                # suite's reverse-in gate).
                _nav = getattr(car, 'bicycle_nav', None)
                _pcs = per_car_state.get(car.uid, {})
                api.update_state({
                    'frame': frame,
                    'time': _sim_steps_total * dt_fixed,
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
                    # Vertical level of the current segment (0 = ground,
                    # 1 = bridge): the Godot renderer z-orders the car
                    # above its own deck but below any higher level, so a
                    # ground car disappears under a bridge.
                    'level': network.segments[car.seg_idx].level,
                    'progress': car.progress,
                    'segment_length': network.segments[car.seg_idx].length,
                    'forward': car.forward,
                    'distance_to_junction': _distance_to_junction(car, network),
                    'on_road': car.is_on_road(network),
                    # Dashboard lamps for the cockpit controller (tools/controller.py)
                    'braking': bool(getattr(car, '_braking', False)),
                    'accelerating': bool(getattr(car, '_accelerating', False)),
                    'wrong_side': _pcs.get('on_wrong_side', False),
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
                    # Per-car counters (each new car starts at zero).
                    'validator_violations': validator.count(car),
                    'hazard': bool(getattr(car.driver, 'hazard', False)),
                    'hazard_reason': getattr(car.driver, 'hazard_reason', ''),
                    'frozen': frozen,
                    'blinker_left': bool(
                        getattr(car.driver, 'blinker_left', False)),
                    'blinker_right': bool(
                        getattr(car.driver, 'blinker_right', False)),
                    'lane_guard_stats': lane_guard.stats(car),
                    # Multi-car: display color + the full car list (the
                    # Godot frontend prefers `cars` over these fields).
                    'color': car.color,
                    'cars': cars_payload,
                    # (The breadcrumb trail is NOT exported here: it is a
                    # pure visual and the Godot frontend records it client-
                    # side from the x/y/heading samples.)
                    'flags': _flags_payload(_tf_p),
                    # Short HUD label (e.g. "5/21") set via POST /label.
                    'hud_label': _tf_p.hud_label if _tf_p else None,
                    'camera_x': camera.x,
                    'camera_y': camera.y,
                    'camera_zoom': camera.zoom,
                })

    if smoke_test_frames:
        if cars:
            _sc = cars[_follow_uid] if _follow_uid in cars else next(
                iter(cars.values()))
            print(f"Smoke test OK: {frame} frames, {len(cars)} car(s), "
                  f"primary #{_sc.uid} at ({_sc.x:.0f}, {_sc.y:.0f}), "
                  f"speed={_sc.speed:.1f} m/s, zoom={camera.zoom:.2f}")
        else:
            print(f"Smoke test OK: {frame} frames, no car on map, "
                  f"zoom={camera.zoom:.2f}")
        return

    sys.exit()


if __name__ == "__main__":
    frames = 0
    if "--smoke" in sys.argv:
        idx = sys.argv.index("--smoke")
        frames = int(sys.argv[idx + 1]) if idx + 1 < len(sys.argv) else 300
    main(smoke_test_frames=frames)
