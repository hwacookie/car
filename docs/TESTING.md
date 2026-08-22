# How Testing Works Here (via the REST Server)

This project has no in-process unit-test harness for driving behavior -
instead, the running game exposes a REST API, and test scripts drive the
actual game (actual physics, actual rendering, actual road network) from
the outside, exactly like a human or another program would. This
document explains that workflow end-to-end.

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

The game itself also spawns deterministically (`--start <name>`, otherwise
the map's first named start point). A random spawn means nothing before the
first teleport is reproducible, and a crash on startup is a different crash
each time.

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

### Random-location mode

```bash
python tests/test_turning.py --random
```

Teleports to random points on whichever map is actually loaded (works
against real OSM data too, unlike the deterministic named-start-point
suite) and repeats the same monitoring at a few speeds. Useful as a
broader smoke test once the deterministic suite passes.

## 3. The test map (`src/test_maps.py`)

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

## 4. Showing which test is running in the HUD

`tests/test_turning.py` also calls `POST /label` (see `docs/REST_API.md`)
with a short number identifying the current test's map tile, shown
top-right of the game window below the minimap - purely a visual aid
so a human watching the window can tell at a glance which tile/scenario
is currently being driven. `START_POINT_NUMBER` in `test_turning.py`
maps each start point name to that number (1 = top-left tile of the
minimap, counted left-to-right then top-to-bottom - note this is the
*visual* minimap position, which is vertically flipped from the map's
internal world-grid row; see the comment above `START_POINT_NUMBER` for
why).

## 5. Other test scripts

- `tests/test_api.py` - basic REST API smoke tests (health check,
  control endpoints respond, etc.) - not scenario/physics testing.

## 6. Typical debugging loop

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
</content>
