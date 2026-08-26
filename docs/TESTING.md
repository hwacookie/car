# How Testing Works Here

Testing has two layers:

1. **Headless pytest suites** (`tests/test_obstacles.py`,
   `tests/test_road_network.py`) - no game process, no window, no REST
   server: they build the road network and car directly in-process and
   assert on placement rules, collision geometry, stop-on-contact
   physics, layout save/load, and the obstacle REST handlers (via
   Flask's test client). They run in seconds and are the first thing to
   run after a change:
   ```bash
   .venv/bin/python -m pytest tests/test_obstacles.py tests/test_road_network.py -q
   ```
2. **Live e2e suite** (the rest of this document) - the running game
   exposes a REST API, and test scripts drive the actual game (actual
   physics, actual rendering, actual road network) from the outside,
   exactly like a human or another program would: `tests/test_turning.py`
   for the turning scenarios, `scripts/verify_obstacles_live.py` for the
   obstacle stop-on-contact scenario.

See `docs/REST_API.md` for the full endpoint reference; this document
focuses on the *testing workflow* itself.

## 1. Start the game with the API enabled

```bash
python -m src.main --map basic --api
```

- `--map basic` loads the synthetic **test map** (`src/test_maps.py`) -
  a small, fully known/deterministic road network purpose-built for
  testing every kind of turn/junction (see "The test map" below).
  Without `--map`, the game loads real OSM data instead, which is fine
  for manual play but not useful for deterministic tests (no fixed,
  named start points).
- `--api` starts a Flask server on `http://localhost:5000` in a
  background thread, alongside the normal Pygame window/loop.
- The game window and the API run **simultaneously** - a test script
  hitting the API is really just remote-controlling (and reading state
  from) the same live game a human would see on screen.

### Headless vs. visible

- **Default: visible.** Just omit `SDL_VIDEODRIVER` - the game window
  opens normally and updates in real time while the test script drives
  it via the API. This is genuinely useful for debugging: you can
  literally watch the car take a turn while a test script asserts on
  the exact same state.
- **Use `SDL_VIDEODRIVER=dummy` ONLY when testing in a build system
  (CI) or when the user explicitly asks for it.** Do not reach for it
  by default on a developer machine:
  ```bash
  SDL_VIDEODRIVER=dummy python -m src.main --map basic --api
  ```

Either way, the API and test behavior are identical - `SDL_VIDEODRIVER`
only affects whether Pygame renders to a real window.

## 2. Run the test suite against it

In a second terminal (the game keeps running in the first):

```bash
python tests/test_turning.py
```

This is the main test script. It:
1. Confirms the API is reachable (`GET /health`).
2. Fetches the map's named start points (`GET /start_points`).
3. For each test scenario: teleports the car to a known start point
   (`POST /teleport {"start_point": ...}`), arms a turn signal or none
   (`POST /control {"blinker_left"/"blinker_right": true}`), holds the
   accelerator down (`POST /control {"accelerate": true}`), and then
   polls `GET /state` every ~50ms for up to 15s, watching for:
   - **off-road violations** (`state['on_road'] == False`)
   - **instant heading snaps** (>30° heading change in one frame - a
     physics bug, not a real turn)
   - whether the car actually changed to the expected next road segment
4. Prints a pass/fail summary, and on failure saves a screenshot via
   `GET /screenshot` to `/tmp/violation_<name>_<timestamp>.png` so you
   can see exactly what went wrong.

### Selecting scenarios

```bash
python tests/test_turning.py --tests 3            # one, by number
python tests/test_turning.py --tests 3,7-9        # several, or a range
python tests/test_turning.py --tests y_from_stem  # every scenario there
python tests/test_turning.py --failed             # whatever failed last run
python tests/test_turning.py --failfast           # stop at the first failure
```

Numbering is preserved when filtering, so "TEST 7/18" always identifies the
same scenario - the point being to jump straight back to one that failed.

Test maps do NOT auto-spawn a car: the window opens with just the map
(`GET /state` reports `has_car: false`) and the suite's setup teleport
creates the car at the scenario's start point before any driving. An
explicit `--start <name>` still places a car at that named point (e.g.
for manual observation); real OSM data keeps its random spawn.

### Running a single scenario

Debugging one specific case is much faster than the whole suite:

```bash
python tests/test_turning.py --only corner_right_entry right 80
```

`--only <start_point> <direction> <speed_kmh>` teleports straight to
that one start point, arms that one turn direction (or `straight`), and
monitors just that one turn. `speed_kmh` is only "how fast to get up to
before we start watching" (the driver always tries to accelerate to top
speed and brakes only as needed for the upcoming turn - see
the speed profile in `src/bicycle_nav.py`, which derives cornering speed
from the curvature of the driving line); it does not force a specific
cruising or cornering speed.

### Cockpit controller (`tools/controller.py`)

The driver's cockpit is a **separate window** (its own process) that talks
to the game over the same REST API - the game window stays a pure world
view. It shows the dashboard (speed, lamps, OFF-ROAD / WRONG-SIDE warnings,
fed by `GET /state`) plus one of two panels:

```bash
python tools/controller.py                 # TEST mode (default): runs the suite
python tools/controller.py --tests 1-3     # ...but only those scenarios
python tools/controller.py --drive         # DRIVE mode: manual driving console
```

- **TEST mode** shows a row of numbered scenario buttons (1..N, same
  numbering as `--tests`). Clicking a button - or typing its number and
  pressing Enter - **aborts the current test and starts that one**, then
  continues with the rest. Buttons turn green/red as scenarios pass/fail;
  aborted runs are reported separately in the summary and never recorded in
  `turning_results.json`. The suite output goes to the console exactly as
  with `test_turning.py`, so the usual `| tee /tmp/test_run.log` workflow
  applies.
- **DRIVE mode** is a manual driving console for FREE mode: hold-buttons
  for steering/gas/brake plus one-shot blinker stalks, hazard and U-turn -
  mouse or keyboard (arrows/WASD, Q/E, H, U). `F` switches the game between
  AI (BICYCLE) and manual (FREE), like TAB in the game window. Both panels
  feed the same `POST /control` channel the test runner uses.

`python tests/test_turning.py` remains the headless/CI way to run the suite;
the cockpit is the visible, interactive one.

### Random-location mode

```bash
python tests/test_turning.py --random
```

Teleports to random points on whichever map is actually loaded (works
against real OSM data too, unlike the deterministic named-start-point
suite) and repeats the same monitoring at a few speeds. Useful as a
broader smoke test once the deterministic suite passes.

## 3. Pass criteria (deterministic scenarios)

A deterministic scenario is a tuple `(start_point, direction, target_speed,
expected_end_segment, duration)` from `DETERMINISTIC_TESTS`. The runner
teleports a fresh car to the MID point of the start segment, accelerates it
to speed, signals the turn at the junction, then monitors the drive. A
scenario **passes** only if ALL of these hold:

1. **Arrival = stop AT the flag.** The car reaches the 50% point of
   `expected_end_segment` (the red end flag) and comes to REST there: either
   it stops within `STOP_AT_FLAG_TOLERANCE_M` (8 m) before the flag, or it
   crosses the flag at crawl speed (≤ `FLAG_CRAWL_KMH`, 5 km/h) and then
   stops. Driving PAST the flag while moving is a FAILURE ("drove past the
   end flag") - parking at the destination is part of this test, not an
   afterthought.
2. **Correct route.** The car ends on exactly `expected_end_segment` (a left
   turn that actually went right fails even if it "arrives" somewhere).
3. **No off-road.** No validator violations (the car must stay on the
   carriageway). Wrong-side driving (crossing the centerline into the
   oncoming lane) is REPORTED but deliberately NOT a failure condition:
   some maneuvers inherently cross the centerline - an on-site U-turn has
   to, by definition (DRIVING_MANEUVERS.md §5: crossing the Mittellinie is
   "erlaubt und erwartet", LaneGuard is suppressed for such maneuvers).
   A blanket wrong-side failure rule would make those scenarios unpassable.
   If a specific scenario should treat wrong-side as fatal, that must be an
   explicit per-scenario flag - not the default.
4. **No instant heading snaps.** ≤ 30° per frame (physically impossible
   rotation).
5. **No teleport/jump.** Movement between polls is physically plausible:
   distance ≤ speed × time + margin.
6. **Game stays alive.** The API answers throughout the scenario.
7. **Within the window.** Arrival within the scenario's monitor window
   (default 30 s; `s_curve` and roundabout 40 s - see the table). Passing
   tests exit on arrival, so the window only bounds how long a FAILING run
   takes.

User-aborted runs (cockpit jump) are neither pass nor fail: they are
reported separately and not persisted to `tests/turning_results.json`.

Criterion 1 is strict on purpose, and it is satisfied by the CAR itself: the
red flag is the nav's destination (`BicycleNav.set_destination`), so the car
parks at the flag with the documented parking approach (parking ramp + kerb
drift + stop). The runner's anticipatory brake latch is only a safety net
for when that approach fails. If the car crosses the flag anyway, that is a
real driving defect - e.g. the nav ignoring the hard brake while in pull-over
mode (this is exactly what used to happen on `s_curve`, which parked at the
dead end beyond the flag) - not a test artifact.

## 4. The test map (`src/test_maps.py`)

`build_basic_test_map()` builds a small synthetic grid (see its
docstring for the full layout) with one specific, deliberately named
test scenario per tile: a plain straight road, a 90° corner (both
directions), a T-junction, a Y-intersection, a 4-way crossroads, a
one-way street through a junction, an S-curve, a dead end, a tight
hairpin, a wide sweeping curve, and a roundabout.

Every scenario has one or more **named start points**
(`RoadNetwork.start_points`, set up via `MapBuilder.start(name,
node_id)`), each a fixed `(x, y, heading, segment, forward)` tuple -
that's what makes `--only <start_point> ...` and the deterministic
suite reproducible: no randomness, no "it happened to spawn somewhere
different this time."

`GET /start_points` returns all of them (name → position/heading/
segment/forward) - handy for exploring what's available:

```bash
curl -s http://localhost:5000/start_points | python3 -m json.tool
```

## 5. Showing which test is running in the HUD

`tests/test_turning.py` also calls `POST /label` (see `docs/REST_API.md`)
with the **current test number** (`"10/15"`), shown top-right of the game
window below the minimap - purely a visual aid so a human watching the
window can tell at a glance which scenario is currently being driven.
Standalone runs (no suite context) fall back to the start point's map-tile
number: `START_POINT_NUMBER` in `test_turning.py` maps each start point
name to that number (1 = top-left tile of the minimap, counted left-to-right
then top-to-bottom - note this is the *visual* minimap position, which is
vertically flipped from the map's internal world-grid row; see the comment
above `START_POINT_NUMBER` for why).

## 6. Other test scripts and suites

- **`tests/test_obstacles.py`** (headless pytest) - the obstacle system:
  placement validation (paved-area only; lane-direction alignment
  including corners and the roundabout ring), SAT collision geometry,
  stop-on-contact physics (full braking, no interpenetration, plus a
  high-speed approach with per-substep heading drift as regression for
  the live pass-through bug), layout save/load and map scoping, the
  `/obstacles` REST handlers via Flask's test client, and headless
  palette drag handling. No game process needed:
  `.venv/bin/python -m pytest tests/test_obstacles.py -q`.
- **`tests/test_road_network.py`** (headless pytest) - road network /
  smoothed geometry unit tests.
- **`scripts/verify_obstacles_live.py`** (live, REST-driven) - places a
  parked-car obstacle in the lane of the running game via `POST
  /obstacles`, drives into it, and asserts the car stops on contact
  without interpenetrating or passing through. Run against a fresh
  `python -m src.main --map basic --api` process.
- `tests/test_api.py` - basic REST API smoke tests (health check,
  control endpoints respond, etc.) - not scenario/physics testing.

## 7. Typical debugging loop

This is the actual workflow used throughout this project's development:

1. Start the game (visible, so you can watch): `python -m src.main --map basic --api`
2. Run one failing scenario: `python tests/test_turning.py --only <name> <direction> <speed>`
3. Watch the window while it runs, and/or inspect the printed physics
   debug output (turn planning, arc validation, off-road/teleportation
   watchdog messages - see `PhysicsValidator`, `LaneGuard` and
   `BicycleNav` in `src/`).
4. If it fails, the violation screenshot in `/tmp/` plus `GET /state`
   polled by hand (`curl http://localhost:5000/state`) is usually enough
   to pin down *where* and *at what exact frame* things went wrong.
5. Fix the code, restart the game process (state like cached geometry
   is built once at startup, so a code change needs a fresh process),
   and re-run the same `--only` scenario before moving on to the full
   suite.

## Notes / gotchas

- The game process must be restarted after any code change - there's no
  hot-reload; `--only` runs are fast specifically so this loop stays
  quick.
- `POST /reset` clears all control inputs (accelerate/brake/blinkers)
  back to `False`; `test_turning.py` calls this before every test.
- Pressing **ESC** in a visible game window triggers an emergency stop
  (sets speed to 0) - useful for humans, but during an automated test
  run this can interact badly with an in-progress turn plan (a stale
  plan targeting nonzero speed can raise a teleportation-watchdog
  exception and kill the game process). Avoid touching the window's
  keyboard focus while a test script is actively driving it.
- **`GET /screenshot` is unreliable while the game window is not
  frontmost on macOS.** When the window is occluded, SDL stops
  presenting and the readback can come back as a black/partial frame
  (stray pixel columns, missing roads) that looks like a rendering bug
  but isn't - the on-screen window is fine. If you need to verify
  pixels programmatically, run the game with `SDL_VIDEODRIVER=dummy`
  (direct CPU-surface readback) instead of screenshotting an occluded
  visible window.
</content>
