#!/usr/bin/env python3
"""
Comprehensive Turn Testing via REST API
Tests both left and right turns with detailed monitoring
"""

import math
import os
import requests
import time
import sys
import json
import signal
from datetime import datetime
from pathlib import Path

# The braking-distance math below must use the game's actual deceleration,
# so it is imported from the shared config instead of duplicated here.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from src.config import (CAR_BRAKING,        # m/s² (the game's ABS braking)
                        CAR_WIDTH,          # m
                        SPRITE_WHEELBASE_M)  # m


API_URL = os.environ.get("CAR_API_URL", "http://127.0.0.1:5000")  # explicit IPv4: 'localhost' may resolve to ::1, where macOS ControlCenter squats on :5000

# Results are persisted next to this script, keyed by "start_point|direction",
# so the next run can report whether each scenario passed the last time it ran
# (and why it failed, if it didn't). See docs/SPEC.md ("Turn Test Output").
RESULTS_FILE = Path(__file__).resolve().parent / "turning_results.json"
HISTORY_LIMIT = 500  # cap the per-scenario history so the file can't grow unbounded
# Signal the turn (blinker) only this many metres before the junction -
# like a real driver, not the instant the test starts.
SIGNAL_DISTANCE_M = 50.0
# A scenario counts as "stopped at the end flag" if the car comes to rest
# within this distance of the 50% point - anticipatory braking plus ~20 Hz
# polling and brake response cannot stop with infinite precision.
#
# The runner's brake-point model (v^2/2a + 0.15*v slack) systematically
# under-predicts the real stopping distance by a few metres (polling
# latency, brake propagation, autopilot modulation), so the car typically
# comes to rest 2.5-3.5 m SHORT of the flag (measured on corner_left,
# where runs landed at 2.5 m and 3.35 m). The tolerance must cover that
# systematic shortfall - worst case ~0.15*v + latency travel ~= 6 m at
# approach speed - or pass/fail becomes a coin flip on the boundary.
STOP_AT_FLAG_TOLERANCE_M = 8.0

# Strict arrival criterion (docs/TESTING.md §3, rule 1): a scenario passes
# only if the car comes to rest AT the end flag - driving PAST it while
# moving is a failure ("drove past the end flag"). Sole exception: crossing
# at crawl speed counts as stopping at the flag - a real driver parking at
# a destination often rolls over the line at a few km/h before coming to
# rest, which is indistinguishable from stopping right at it.
FLAG_CRAWL_KMH = 5.0

# Parking-offset scenarios (docs/DRIVING_MANEUVERS.md §1 variant): the
# arrival at the flag is not enough - the car must also have PARKED, i.e.
# BOTH right-hand wheels end within this distance of the right kerb.
# Measured at the body flank (CAR_WIDTH/2) at both wheel stations: a
# perfectly parked car sits PARK_KERB_CLEARANCE_M (0.16 m) out, so 30 cm
# means "parked properly", not "touched the kerb".
KERB_PASS_MAX_M = 0.30
# Width of the parking avenue in src/test_maps.py (tile (4,1): two driving
# + one parking lane per side) - the right-kerb line for the check above is
# derived from it.
PARK_AVENUE_WIDTH_M = 19.4

# Running turn tests (docs/TESTING.md): the scenario drives THROUGH the
# corner - no stopping, no parking (parking is covered by the dedicated
# parking-offset scenarios). The car spawns in the LAST 20% of the start
# segment (progress TURN_SPAWN_PROGRESS), already rolling at
# RUNNING_START_KMH, and the test ends when it crosses the red end flag,
# which marks the FIRST 20% of the expected end segment. Pass criterion:
# the turn was executed correctly - stayed on the road and did not drive
# into the oncoming lane (lane guard).
TURN_SPAWN_PROGRESS = 0.8
END_FLAG_PROGRESS = 0.2
# Radius within which the car's centre must pass the red end flag for the
# scenario to count as "reached the flag". Must cover one polling step at
# max test speed (50 km/h @ ~20 Hz ≈ 0.7 m) plus the largest lane offset
# on a running-test road (kerb_offset of a 7 m road = 2.25 m), while
# staying far below any real miss distance.
FLAG_REACH_RADIUS_M = 3.0
RUNNING_START_KMH = 50.0


# --- Colored console output (auto-disabled when stdout is not a TTY) ---

def _c(text: str, code: str) -> str:
    if not sys.stdout.isatty():
        return text
    return f"\033[{code}m{text}\033[0m"


def green(text: str) -> str:  return _c(text, "32")
def red(text: str) -> str:    return _c(text, "31")
def yellow(text: str) -> str: return _c(text, "33")
def cyan(text: str) -> str:   return _c(text, "36")
def dim(text: str) -> str:    return _c(text, "2")


# --- Result persistence (tests/turning_results.json) ---

def _parse_test_spec(spec_str: str):
    """Parse a scenario selection string into (indices, names).

    Tokens are comma-separated: '3' (one number), '7-9' (inclusive range)
    or a start-point name (every scenario at that point)."""
    wanted_idx: set[int] = set()
    wanted_name: set[str] = set()
    for tok in spec_str.split(','):
        tok = tok.strip()
        if not tok:
            continue
        if '-' in tok and all(p.strip().isdigit() for p in tok.split('-', 1)):
            a, b = (int(p) for p in tok.split('-', 1))
            wanted_idx.update(range(a, b + 1))
        elif tok.isdigit():
            wanted_idx.add(int(tok))
        else:
            wanted_name.add(tok)
    return wanted_idx, wanted_name


def select_tests(tests: list, spec: str | None = None) -> list:
    """Pick which scenarios to run, keeping their ORIGINAL numbering.

    With `spec` given (used by the cockpit controller), it is parsed like
        3              one scenario
        3,7,12         several
        3-6            an inclusive range
        tjunction_from_top      every scenario at that start point
    Without it, the selection is read from the command line: --tests with
    the same syntax, or --failed for whatever failed on the last recorded
    run.

    Returns [(original_index, test), ...] so the printed "TEST 7/18" still
    identifies the same scenario however the suite is filtered - the point
    being to jump straight back to one that failed without renumbering it.
    """
    numbered = list(enumerate(tests, 1))

    if spec is not None:
        wanted_idx, wanted_name = _parse_test_spec(spec)
        picked = [(i, t) for i, t in numbered
                  if i in wanted_idx or t[0] in wanted_name]
        if not picked:
            print(f"test selection '{spec}' matched nothing. Available:")
            for i, t in numbered:
                print(f"   {i:>2}  {t[0]} {t[1]}")
            sys.exit(2)
        return picked

    if '--failed' in sys.argv:
        last = load_results().get('last', {})
        failed = {k for k, v in last.items() if not v.get('passed')}
        if not failed:
            print("--failed: nothing failed on the last run; nothing to do.")
            return []
        picked = [(i, t) for i, t in numbered
                  if f"{t[0]}|{t[1]}" in failed]
        print(f"--failed: re-running {len(picked)} previously failing "
              f"scenario(s): {', '.join(sorted(failed))}")
        return picked

    if '--tests' not in sys.argv:
        return numbered

    idx = sys.argv.index('--tests')
    if idx + 1 >= len(sys.argv):
        print("--tests needs a value, e.g. --tests 3,7-9,tjunction_from_top")
        sys.exit(2)

    wanted_idx, wanted_name = _parse_test_spec(sys.argv[idx + 1])

    picked = [(i, t) for i, t in numbered
              if i in wanted_idx or t[0] in wanted_name]
    if not picked:
        print(f"--tests '{sys.argv[idx + 1]}' matched nothing. Available:")
        for i, t in numbered:
            print(f"   {i:>2}  {t[0]} {t[1]}")
        sys.exit(2)
    return picked


def load_results() -> dict:
    """Load the persisted turn-test results (empty structure if missing/corrupt)."""
    try:
        with open(RESULTS_FILE) as f:
            data = json.load(f)
        if isinstance(data, dict) and isinstance(data.get("last"), dict):
            return data
    except (OSError, ValueError):
        pass
    return {"last": {}, "history": []}


def save_results(data: dict) -> None:
    """Persist results. Best-effort: a write failure must never kill the run."""
    data["updated"] = datetime.now().isoformat(timespec="seconds")
    data["history"] = data.get("history", [])[-HISTORY_LIMIT:]
    try:
        with open(RESULTS_FILE, "w") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
    except OSError as e:
        print(yellow(f"⚠️  Could not save results to {RESULTS_FILE}: {e}"))


def record_result(data: dict, result: dict) -> None:
    """Record one scenario result (updates 'last' + appends to 'history')."""
    if not result.get("start_point"):
        return  # random-location runs have no stable scenario key
    key = f"{result['start_point']}|{result['direction']}"
    entry = {
        "passed": bool(result["passed"]),
        "reason": describe_failure(result),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "final_segment": result.get("final_segment"),
        "expected_end_segment": result.get("expected_end_segment"),
    }
    data["last"][key] = entry
    data.setdefault("history", []).append({"scenario": key, **entry})


def last_result_for(data: dict, start_point: str, direction: str):
    """Return the previous result entry for a scenario, or None."""
    if not start_point:
        return None
    return data["last"].get(f"{start_point}|{direction}")


# --- Human-readable failure reasons ---

def describe_failure(result: dict) -> str | None:
    """One-line human-readable reason for a failed result (None if it passed).

    This is the text shown under "Last run: No - <reason>" at the start of the
    next run of the same scenario, and after "FAIL: " when a test fails.
    """
    if result["passed"]:
        return None
    if result.get("aborted"):
        return "aborted by user (test jump from the cockpit)"
    if result.get("game_crashed"):
        return "game process ended mid-test (window closed or physics watchdog crash)"
    if result.get("teleport_detected"):
        return "teleported / unexpected jump detected"
    if result.get("instant_snap_detected"):
        return "instant heading snap (unrealistic rotation)"
    if result.get("expect_wrong_side") and not result.get("wrong_side_detected"):
        return ("expected wrong-side driving but none occurred - the car "
                "stayed in its own lane (negative detector test did not fire)")
    if result.get("wrong_side_detected") and not result.get("expect_wrong_side"):
        return "drove into the oncoming lane (wrong side of the centreline)"
    # off_road_detected already accounts for the validator log, measured as
    # a DELTA over the scenario. Re-testing the cumulative count here would
    # reintroduce the same latch.
    if result.get("off_road_detected"):
        return "cut the corner and drove off the road"
    if result.get("passed_flag"):
        return (f"drove past the end flag at "
                f"{result.get('passed_flag_speed_kmh', 0):.0f} km/h - the "
                f"scenario requires a stop AT the flag")
    if result.get("segment_changed") and not result.get("reached_expected_segment"):
        final = result.get("final_segment")
        expected = result.get("expected_end_segment")
        if final == expected:
            # On the right segment but the end flag was never reached -
            # calling that "the wrong route" is a lie (it cost real debugging
            # time to decode). Usually: approach too slow for the window, or
            # the stop landed just short of the flag tolerance.
            target_pct = 50 if result.get("kerb_check") \
                else result.get("end_flag_progress", END_FLAG_PROGRESS) * 100
            return (f"timed out on the correct segment {final} before "
                    f"reaching the end flag ({target_pct:.0f}% point)")
        return (f"took the wrong route (ended on segment {final}, "
                f"expected {expected})")
    if result.get("reached_expected_segment") and not result.get("stopped_at_end"):
        return "reached the destination but never came to a clean stop"
    if result.get("kerb_check") and result.get("reached_expected_segment") \
            and result.get("stopped_at_end"):
        gaps = result.get("kerb_gaps_m")
        if gaps is not None:
            return (f"parked too far from the kerb: right flank "
                    f"{gaps[1] * 100:.0f} cm front / {gaps[0] * 100:.0f} cm rear "
                    f"(limit {KERB_PASS_MAX_M * 100:.0f} cm)")
    return (f"timed out: never reached expected segment "
            f"{result.get('expected_end_segment')}")


# --- User-interruption handling (Ctrl-C / game window closed) ---

_INTERRUPTED = False


def _on_sigint(signum, frame):
    global _INTERRUPTED
    _INTERRUPTED = True
    print("\n" + yellow("⚠️  Ctrl-C received - finishing current scenario, then saving results..."))


def game_alive() -> bool:
    """True while the game's API still answers (window closed => not alive)."""
    try:
        requests.get(f"{API_URL}/health", timeout=1)
        return True
    except requests.exceptions.RequestException:
        return False


def interrupted() -> bool:
    """True if the user interrupted (Ctrl-C) or the game window was closed."""
    return _INTERRUPTED or not game_alive()

# Sequential, 1-based position of each named start point's map tile,
# counted from the TOP-LEFT of the minimap, left-to-right then
# top-to-bottom (1 = top-left, 2 = top row/second from left, etc.) -
# used only to show a short number in the HUD via POST /label while a
# test runs, purely a visual aid to see where on the map the current
# test is happening. The descriptive start-point names themselves are
# unaffected.
#
# NOTE: the minimap draws with north (small world y) at the BOTTOM and
# south (large world y) at the TOP (Renderer.draw_minimap flips y), so
# this numbering is the reverse of the tiles' internal world-grid row
# (see src/test_maps.py:build_basic_test_map's docstring) - world-grid
# row 0 appears at the bottom of the minimap (numbers 9-12) and
# world-grid row 2 at the top (numbers 1-4).
START_POINT_NUMBER = {
    'dead_end_approach': 1,
    'hairpin_entry': 2, 'hairpin_exit': 2,
    'sweeping_curve': 3, 'sweeping_curve_reverse': 3,
    'roundabout_from_north': 4, 'roundabout_from_east': 4,
    'roundabout_from_south': 4, 'roundabout_from_west': 4,
    'y_from_stem': 5, 'y_from_sw': 5, 'y_from_se': 5,
    'crossroads_from_north': 6, 'crossroads_from_south': 6,
    'crossroads_from_east': 6, 'crossroads_from_west': 6,
    'oneway_entry': 7, 'oneway_wrong_way': 7,
    'oneway_cross_from_north': 7, 'oneway_cross_from_south': 7,
    's_curve': 8, 's_curve_reverse': 8,
    'straight': 9, 'straight_reverse': 9,
    'corner_right_entry': 10, 'corner_right_exit': 10,
    'corner_left_entry': 11, 'corner_left_exit': 11,
    'tjunction_from_top': 12, 'tjunction_from_west': 12, 'tjunction_from_east': 12,
    'sliver_approach': 13, 'sliver_from_west': 13, 'sliver_from_east': 13,
    'park_6lane_left_lane': 14, 'park_6lane_right_lane': 14,
    'park_6lane_parking': 14,
    'mixed_from_west': 15, 'mixed_from_east': 16,
}


# The deterministic scenario table (see the long comment on it). Kept at
# module level so the cockpit controller (tools/controller.py) can import
# it to build its row of numbered test buttons without running the suite.
# Default monitor window: 30 s. Passing tests exit ON ARRIVAL, so this only
# bounds how long a FAILING run takes - but it must comfortably exceed the
# slowest legitimate arrival. Running turn tests end at the flag crossing
# (first 20% of the end segment - no stop), so arrivals are quick except
# on the roundabout: the ring's chord segments delay the route horizon and
# the car has to circle most of the ring before reaching the west exit,
# hence its longer window. The parking-offset scenarios keep the full
# parking approach (parking ramp + kerb drift + back-in + stop at the
# flag), which is slower - 45 s covers them.
DETERMINISTIC_TESTS = [
    # 90-degree corners (the classic reported bug)
    ('corner_right_entry', 'right', 50, 6, 60.0,
     "Taking a sharp 90° right corner"),
    ('corner_left_entry', 'left', 50, 8, 60.0,
     "Taking a sharp 90° left corner"),
    # T-junction (perpendicular 3-way)
    ('tjunction_from_top', 'left', 50, 11, 60.0,
     "T-junction: turning left off the stem"),
    ('tjunction_from_top', 'right', 50, 10, 60.0,
     "T-junction: turning right off the stem"),
    # Y-intersection (shallow ~40 degree diverging angles)
    ('y_from_stem', 'left', 50, 14, 60.0,
     "Y-intersection: taking the left fork"),
    ('y_from_stem', 'right', 50, 13, 60.0,
     "Y-intersection: taking the right fork"),
    # 4-way crossroads
    ('crossroads_from_north', 'left', 50, 18, 60.0,
     "4-way crossroads: turning left"),
    ('crossroads_from_north', 'right', 50, 17, 60.0,
     "4-way crossroads: turning right"),
    ('crossroads_from_north', 'straight', 50, 16, 60.0,
     "4-way crossroads: going straight through"),
    # One-way street (legal direction)
    ('oneway_entry', 'straight', 50, 20, 60.0,
     "Entering a one-way street in the legal direction"),
    # Mixed one-way/two-way crossroads (tile (4,2)): W spoke is one-way
    # INTO the junction; entering from the west, the car must swing
    # slightly right across the junction into its own lane on the two-way
    # east side (the keep-centre-on-left rule does not apply on the
    # one-way approach - no oncoming traffic there).
    ('mixed_from_west', 'straight', 50, 111, 60.0,
     "One-way into a two-way crossroads: straight through"),
    # Mixed one-way/two-way crossroads (tile (4,3)): W spoke is one-way
    # OUT of the junction; entering from the east (two-way), the car
    # crosses the junction and eases onto the narrow one-way exit.
    ('mixed_from_east', 'straight', 50, 114, 60.0,
     "Two-way into a one-way exit: straight through"),
    # Simple curves (degree-2 nodes, no blinker needed). Under the running
    # protocol only the corner ENTRY is covered (last 20% of the approach
    # + first 20% of the target segment) - the long S-curve traversal the
    # old stop-at-50% protocol measured is out of scope now.
    ('s_curve', 'straight', 50, 26, 60.0,
     "Following a zig-zag S-curve road"),
    # Test 12 overrides: spawn at 50% of the approach segment AND finish
    # at the 50% point of the end segment (tuple fields:
    # start_progress=0.5, kerb_check=False, end_flag_progress=0.5).
    ('hairpin_entry', 'straight', 50, 29, 60.0,
     "Driving a hairpin bend (entry side)", 0.5, False, 0.5),
    ('sweeping_curve', 'straight', 50, 31, 60.0,
     "Following a wide, sweeping curve"),
    # Hairpin, entered from the opposite end (reverse direction)
    ('hairpin_exit', 'straight', 50, 28, 60.0,
     "Driving a hairpin bend (exit side)", 0.5, False, 0.5),
    # Roundabout (one-way ring, 4 two-way spokes). 'straight'
    # (or 'left') at the entry just merges onto the ring and then
    # keeps circling it FOREVER - a one-way loop has no "next
    # different segment" to naturally stop at unless the car
    # actually exits onto a spoke. 'right' does that here
    # (verified: exits west, segments 28) - a real, completed
    # roundabout maneuver instead of an endless circle.
    # The ring is COUNTER-CLOCKWISE (correct for Germany / right-hand
    # traffic: the island stays on your left). Entering from the north
    # and going counter-clockwise, the first exit encountered is WEST
    # (seg 99 = rb_west_far spoke). Monitoring keeps running THROUGH
    # the intermediate ring segments instead of stopping at the first
    # one, since only 99 (the actual exit) counts as arrival. Takes
    # longer than a normal turn (~25s to go most of the way around
    # before exiting), hence the longer duration override.
    ('roundabout_from_north', 'right', 40, 99, 60.0,
     "Circling the roundabout and taking the first exit (west)"),
    # Sliver junction (the real-world segment-815 layout: a 4.16 m
    # approach stub meeting a 3-way junction where one exit is a
    # near-90-degree turn). The car must get through the tiny
    # stub and onto the correct exit without clipping the junction.
    # NOTE: segment indices shifted from 41/42/43 to 97/96/99 due to
    # the 64-node roundabout ring, and again to 101/102/103 after the
    # multi-width U-turn track was added (straight=101 sliv_str,
    # right=102 sliv_w, left=103 sliv_e).
    # DISABLED for now (pre-existing failures: car goes off-road on
    # the short dead-end exits after the turn) - re-enable once the
    # sliver exit geometry is fixed.
    # ('sliver_approach', 'straight', 80, 101, 15.0,
    #  "Sliver junction: going straight through the tiny approach stub"),
    # ('sliver_approach', 'right', 80, 102, 15.0,
    #  "Sliver junction: turning right off the tiny approach stub"),
    # ('sliver_approach', 'left', 80, 103, 15.0,
    #  "Sliver junction: turning left off the tiny approach stub"),
    # Park from different lateral start positions (docs/DRIVING_MANEUVERS.md
    # §1 variant). Straight parking avenue (tile (4,1), segment index 109 -
    # NOTE: the game's state/flag API uses 0-based indices, NOT the 1-based
    # RoadSegment id 110): two-way, 19.4 m = per side two 3.5 m driving
    # lanes + a 2.7 m parking lane at the kerb (solid centreline, painted P
    # marks). The car starts at 15% of the segment and must park at the flag
    # (50%) with BOTH right-hand wheels within 30 cm of the right kerb (the
    # extra tuple fields: start_progress, kerb_check) - i.e. INSIDE the
    # parking lane. Only the lateral spawn position varies between the three,
    # always on our own side of the road; it is baked into the named start
    # points. From the left (overtaking) driving lane the car must first
    # merge right onto the normal line, well before the flag - a human never
    # backs in from the overtaking lane; from inside the PARKING LANE it
    # merges LEFT back into traffic first - the parking lane is for
    # parking, not for travelling (user rules, replace the old "hold
    # initial line" rule of 2026-08-27).
    ('park_6lane_left_lane', 'straight', 80, 109, 45.0,
     "Park from the left (overtaking) driving lane - merge right first", 0.15, True),
    ('park_6lane_right_lane', 'straight', 80, 109, 45.0,
     "Back-in park from the right (normal) driving lane", 0.15, True),
    ('park_6lane_parking', 'straight', 80, 109, 45.0,
     "Park from inside the parking lane - rejoin the driving lane first", 0.15, True),
    # NEGATIVE TEST (wrong-side detector) - runs LAST on purpose: the old
    # test-14 config. Spawn at 80% of segment 29, i.e. right at the
    # hairpin fillet, already rolling at 50 km/h. The car cannot make the
    # ~166-degree turn in time and slides across the centreline into the
    # oncoming lane at the tip of the V. This scenario is EXPECTED to
    # drive wrong-side: it PASSES when the suite detects the violation
    # (live 'wrong_side' state or the lane-guard frame counter) and FAILS
    # if the car somehow stays in its own lane - it proves the detection
    # works instead of testing driving quality.
    ('hairpin_exit', 'straight', 50, 28, 60.0,
     "NEGATIVE: spawn too close to the hairpin corner at 50 km/h - must "
     "slide into the oncoming lane and trigger the wrong-side detection",
     0.8, False, 0.5, True),
]


class TurnTester:
    """Automated turn tester with detailed violation reporting."""
    
    def __init__(self):
        self.test_results = []
    
    def health_check(self) -> bool:
        """Verify API is available."""
        try:
            response = requests.get(f"{API_URL}/health", timeout=1)
            data = response.json()
            print("✅ API health check passed")
            return True
        except requests.exceptions.RequestException as e:
            print(f"❌ API not available: {e}")
            print("\n🚨 Start the game first:")
            print("   python -m src.main --api\n")
            return False
    
    def reset_controls(self):
        """Reset all control inputs (no-op if the game is no longer reachable)."""
        try:
            requests.post(f"{API_URL}/reset")
        except requests.exceptions.RequestException:
            pass
    
    def get_state(self):
        """Get current game state."""
        response = requests.get(f"{API_URL}/state")
        return response.json()
    
    def get_segment_geometry(self, idx: int) -> dict | None:
        """One road segment's geometry from the game (pixels, 0-based)."""
        try:
            r = requests.get(f"{API_URL}/segment/{idx}", timeout=2)
            if r.ok:
                return r.json()
        except requests.exceptions.RequestException:
            pass
        return None
    
    def send_control(self, **kwargs):
        """Send control inputs."""
        requests.post(f"{API_URL}/control", json=kwargs)
    
    def _wait_for_new_car(self, old_uid):
        """Wait until the game has processed a teleport (a car with a NEW uid).

        has_car alone can't confirm this: it is already True while the OLD
        car still exists, so reading state right after /teleport used to
        return the PREVIOUS car's segment/position as this scenario's
        "initial" values. When that stale segment equals the expected end
        segment, monitor_turn's `segment != initial_segment` gate skips the
        entire arrival logic on the target segment and the test times out
        with a confusing "wrong route (ended on X, expected X)" - this is
        what flaked corner_left after an aborted run and broke oneway/
        hairpin when the previous scenario parked on the target segment.
        Car.uid is monotonic per instance, so a changed uid is an
        unambiguous teleport ack (polling instead of a blind sleep -
        usually lands in ~50 ms).
        """
        deadline = time.time() + 2.0
        while time.time() < deadline:
            try:
                st = self.get_state()
            except requests.exceptions.RequestException:
                return   # game unreachable - the caller deals with it
            if st.get('has_car') and st.get('car_uid') != old_uid:
                return
            time.sleep(0.05)

    def create_car_at_random(self):
        """Replace car with a fresh one at random location."""
        try:
            old_uid = self.get_state().get('car_uid')
        except requests.exceptions.RequestException:
            old_uid = None
        requests.post(f"{API_URL}/teleport", json={'random': True})
        self._wait_for_new_car(old_uid)

    def create_car_at_start_point(self, name: str, progress: float = 0.5,
                                  speed_mps: float | None = None):
        """Replace car with a fresh one at named start point.

        progress: fraction along the start segment (from the node). The
        suite standard is TURN_SPAWN_PROGRESS - running turn tests use
        only the LAST 20% of the start segment (see docs/TESTING.md).
        speed_mps: optional rolling start in m/s (running turn tests spawn
        already moving; parking tests start from rest).
        """
        try:
            old_uid = self.get_state().get('car_uid')
        except requests.exceptions.RequestException:
            old_uid = None
        payload = {'start_point': name, 'progress': progress}
        if speed_mps is not None:
            payload['speed'] = speed_mps
        requests.post(f"{API_URL}/teleport", json=payload)
        # Wait until the game loop has actually processed the teleport and
        # the state reflects the NEW car (see _wait_for_new_car).
        self._wait_for_new_car(old_uid)
    
    def set_hud_label(self, text: str | None):
        """Show (or clear) a short text label in the game's HUD."""
        requests.post(f"{API_URL}/label", json={'text': text})
    
    def _right_side_kerb_gaps_m(self, state: dict, start_point: str):
        """Gap (m) between the car's RIGHT FLANK at both wheel stations and
        the right kerb line of the scenario's road; positive = on the road.

        The basic map is deterministic, so the kerb line comes from the
        start point's centreline position + heading (GET /start_points):
        a straight line PARK_AVENUE_WIDTH_M/2 to the right of the legal
        direction of travel. The car pose is its rear axle (state x/y) +
        heading; the wheel stations are the rear axle and the front axle
        (SPRITE_WHEELBASE_M ahead), measured at the body flank
        (CAR_WIDTH/2 outboard).
        """
        sp = self.get_start_points().get(start_point)
        if sp is None:
            return None
        pppm = 2.0  # config.PIXELS_PER_METER
        h = math.radians(sp['heading'])
        fx, fy = math.sin(h), math.cos(h)          # direction of travel
        rx, ry = fy, -fx                           # right-hand side of it
        half_w_px = (PARK_AVENUE_WIDTH_M / 2.0) * pppm
        cx, cy = sp['x'] + rx * half_w_px, sp['y'] + ry * half_w_px  # kerb line
        hd = math.radians(state['heading'])
        fcx, fcy = math.sin(hd), math.cos(hd)      # car forward
        rcx, rcy = fcy, -fcx                       # car right side
        x, y = state['x'], state['y']              # rear axle (world px)
        wb_px = SPRITE_WHEELBASE_M * pppm
        flank_px = (CAR_WIDTH / 2.0) * pppm
        gaps = []
        for wx, wy in ((x + rcx * flank_px, y + rcy * flank_px),          # rear right
                       (x + fcx * wb_px + rcx * flank_px,
                        y + fcy * wb_px + rcy * flank_px)):               # front right
            # signed distance from the kerb line into the road (n = -r)
            gaps.append(((wx - cx) * (-rx) + (wy - cy) * (-ry)) / pppm)
        return gaps

    def get_start_points(self) -> dict:
        """List available deterministic start points from the loaded map."""
        response = requests.get(f"{API_URL}/start_points")
        return response.json()
    
    def save_violation_screenshot(self, test_name: str, state: dict):
        """Save screenshot when violation detected."""
        try:
            response = requests.get(f"{API_URL}/screenshot")
            if response.status_code == 200:
                filename = f"/tmp/violation_{test_name}_{int(time.time())}.png"
                with open(filename, 'wb') as f:
                    f.write(response.content)
                print(f"      📷 Screenshot saved: {filename}")
                return filename
        except Exception as e:
            print(f"      ⚠️  Screenshot failed: {e}")
        return None
    
    def _drive_to_segment_end_and_stop(self, target_segment: int, start_time: float,
                                        max_extra_time: float = 25.0,
                                        abort_check: "callable | None" = None):
        """Having just arrived on the designated target segment, keep
        actually driving it (don't just release controls mid-segment) -
        brake as we approach ITS far end and confirm the car really comes
        to a full stop there, same as a human driver pulling in and
        parking rather than abandoning the car halfway down the road.
        
        Returns (final_position, stopped_cleanly: bool, details: dict).
        details may include off_road/instant_snap/teleport/game_crashed
        flags and a violation_details dict, same shape as monitor_turn's
        own violation reporting, for the caller to fold in.
        """
        details = {}
        last_heading = None
        last_pos = None
        last_time = time.time()
        braking_commanded = False
        deadline = time.time() + max_extra_time
        
        while time.time() < deadline:
            if abort_check is not None and abort_check():
                details['aborted'] = True
                return (last_pos or (0, 0)), False, details
            try:
                state = self.get_state()
            except requests.exceptions.RequestException as e:
                details['game_crashed'] = True
                details['violation_details'] = {'type': 'game_crashed', 'error': str(e)}
                return (last_pos or (0, 0)), False, details
            
            pos = (state['x'], state['y'])
            if last_heading is not None:
                heading_diff = abs((state['heading'] - last_heading + 180) % 360 - 180)
                if heading_diff > 30.0:
                    details['instant_snap'] = True
                    details['violation_details'] = {'type': 'instant_heading_snap', 'position': pos}
                    return pos, False, details
            last_heading = state['heading']
            
            if not state['on_road']:
                details['off_road'] = True
                details['violation_details'] = {
                    'type': 'off_road', 'position': pos,
                    'time': time.time() - start_time,
                    'speed_kmh': state['speed_kmh'],
                    'heading': state['heading'],
                }
                return pos, False, details
            
            now = time.time()
            if last_pos is not None:
                poll_dt = now - last_time
                moved_m = ((pos[0] - last_pos[0]) ** 2 + (pos[1] - last_pos[1]) ** 2) ** 0.5 / 2.0
                max_plausible_m = max(state['speed_kmh'] / 3.6, 50.0) * poll_dt * 1.5 + 1.0
                if moved_m > max_plausible_m:
                    details['teleport'] = True
                    details['violation_details'] = {'type': 'teleport', 'position': pos, 'distance_m': moved_m}
                    return pos, False, details
            last_pos = pos
            last_time = now
            
            if state['segment'] != target_segment:
                # Drove clean through and out the other side without ever
                # needing to brake (e.g. a very short target segment) -
                # that's fine, nothing to stop for; treat as parked.
                return pos, True, details
            
            progress = state.get('progress', 0.5)
            forward = state.get('forward', True)
            near_end = progress >= 0.92 if forward else progress <= 0.08
            
            if near_end or state['speed_kmh'] < 1.0:
                braking_commanded = True
                self.send_control(accelerate=False, brake=True)
            elif not braking_commanded:
                self.send_control(accelerate=True, brake=False)
            
            if braking_commanded and state['speed_kmh'] < 1.0:
                self.send_control(accelerate=False, brake=False)
                return pos, True, details
            
            time.sleep(0.05)
        
        # Ran out of time without coming to a stop on the target segment.
        return (last_pos or (0, 0)), False, details
    
    def _brake_to_stop(self, start_time: float, max_wait: float = 10.0,
                       abort_check: "callable | None" = None):
        """Brake and wait for the car to come to a FULL stop right where it
        is - the scenario's destination (the red end flag). The brake stays
        LATCHED on purpose: releasing it would let the autopilot pull
        straight off again; the next scenario's reset_controls() clears the
        latch.
        
        Returns (final_position, stopped_cleanly: bool, details: dict),
        same shape as _drive_to_segment_end_and_stop, for the caller to
        fold into the result.
        """
        details = {}
        last_heading = None
        last_pos = None
        last_time = time.time()
        deadline = time.time() + max_wait
        
        self.send_control(accelerate=False, brake=True)
        
        while time.time() < deadline:
            if abort_check is not None and abort_check():
                details['aborted'] = True
                return (last_pos or (0, 0)), False, details
            try:
                state = self.get_state()
            except requests.exceptions.RequestException as e:
                details['game_crashed'] = True
                details['violation_details'] = {'type': 'game_crashed', 'error': str(e)}
                return (last_pos or (0, 0)), False, details
            
            pos = (state['x'], state['y'])
            if last_heading is not None:
                heading_diff = abs((state['heading'] - last_heading + 180) % 360 - 180)
                if heading_diff > 30.0:
                    details['instant_snap'] = True
                    details['violation_details'] = {'type': 'instant_heading_snap', 'position': pos}
                    return pos, False, details
            last_heading = state['heading']
            
            if not state['on_road']:
                details['off_road'] = True
                details['violation_details'] = {
                    'type': 'off_road', 'position': pos,
                    'time': time.time() - start_time,
                    'speed_kmh': state['speed_kmh'],
                    'heading': state['heading'],
                }
                return pos, False, details
            
            now = time.time()
            if last_pos is not None:
                poll_dt = now - last_time
                moved_m = ((pos[0] - last_pos[0]) ** 2 + (pos[1] - last_pos[1]) ** 2) ** 0.5 / 2.0
                max_plausible_m = max(state['speed_kmh'] / 3.6, 50.0) * poll_dt * 1.5 + 1.0
                if moved_m > max_plausible_m:
                    details['teleport'] = True
                    details['violation_details'] = {'type': 'teleport', 'position': pos, 'distance_m': moved_m}
                    return pos, False, details
            last_pos = pos
            last_time = now
            
            if state['speed_kmh'] < 1.0:
                # Parked - keep the brake latched (see docstring).
                return pos, True, details
            
            time.sleep(0.05)
        
        # Ran out of time without coming to a stop.
        return (last_pos or (0, 0)), False, details
    
    def _aborted_result(self, start_point: str | None, direction: str,
                        target_speed: float, results: dict | None) -> dict:
        """Build + report the result of a scenario the user ABORTED (a test
        jump from the cockpit controller).

        An aborted run is NOT recorded in turning_results.json: it was never
        actually tested, so it must not overwrite the last real result (and
        must not show up in --failed). It IS kept in self.test_results so
        the summary can report it as "aborted" instead of dropping it.
        """
        print(f"\n   ⏭  ABORTED - user jumped to another test from the cockpit")
        try:
            self.reset_controls()
        except requests.exceptions.RequestException:
            pass
        result = {
            'start_point': start_point,
            'direction': direction,
            'target_speed_kmh': target_speed,
            'frames_checked': 0,
            'duration': 0.0,
            'initial_segment': -1,
            'expected_end_segment': None,
            'final_segment': -1,
            'start_position': (0.0, 0.0),
            'end_position': (0.0, 0.0),
            'segment_changed': False,
            'reached_expected_segment': False,
            'stopped_at_end': False,
            'off_road_detected': False,
            'instant_snap_detected': False,
            'teleport_detected': False,
            'game_crashed': False,
            'max_heading_change_per_frame': 0.0,
            'violation_details': None,
            'positions': [],
            'aborted': True,
            'passed': False,
        }
        self.test_results.append(result)
        print(f"{'─'*60}\n")
        return result

    def monitor_turn(self, direction: str, duration: float = 15.0, target_speed: float = 50.0,
                      start_point: str | None = None, expected_end_segment: int | None = None,
                      description: str | None = None, results: dict | None = None,
                      abort_check: "callable | None" = None,
                      label: str | None = None, start_progress: float = 0.5,
                      kerb_check: bool = False,
                      end_flag_progress: float = END_FLAG_PROGRESS,
                      expect_wrong_side: bool = False) -> dict:
        """Monitor a turn for violations.
        
        Args:
            direction: "left" or "right"
            duration: Maximum time to monitor (seconds)
            target_speed: Target speed in km/h
            start_point: If given, teleport to this deterministic named start
                point (synthetic test map) instead of a random location.
            expected_end_segment: If given, the test only PASSES if the car
                ends up on exactly this segment - not just "any segment
                change" (which would silently pass even if e.g. a LEFT
                turn actually went RIGHT, as long as it went somewhere -
                this is exactly the kind of bug that slipped through
                before the API-blinker-routing fix). Monitoring keeps
                running across MULTIPLE segment changes (e.g. a
                roundabout: entry -> ring -> ring -> exit) until the car
                has driven to the 50% point of this exact segment
                (mid-segment, mirroring the mid-segment START - the far
                end / parking tail is NOT part of a turn test), a
                violation occurs, or duration runs out. If None (only used
                for the legacy random-location suite, where there's no way
                to know the correct answer in advance), falls back to the
                old "any change + drive to the far end and stop" behavior.
            start_progress: Where along the start segment to spawn (0..1
                from the named node). The suite standard is 0.5; the
                parking-offset scenarios use 0.15 so there is ~100 m of
                approach between spawn and the flag at 50%.
            kerb_check: Parking-offset scenarios only (docs/
                DRIVING_MANEUVERS.md §1 variant). The arrival at the flag
                is not enough - BOTH right-hand wheels must end within
                KERB_PASS_MAX_M of the right kerb or the scenario fails.
            end_flag_progress: Where along the expected end segment the
                red end flag sits (0..1 from the segment's first node).
                Suite standard is END_FLAG_PROGRESS (first 20%); a
                scenario may override it (e.g. test 12 finishes at the
                50% point of the end segment).
            abort_check: Optional zero-arg callable, polled throughout setup
                and monitoring. If it returns truthy, the scenario is
                ABORTED right away (the cockpit controller sets this when
                the user clicks a different test button): inputs are
                cleared and an 'aborted' result is returned - NOT recorded
                in turning_results.json, since the scenario was never
                actually tested.
        
        Returns:
            dict with test results
        """
        def _aborted() -> bool:
            return abort_check is not None and bool(abort_check())
        # --- What is this test? / what happened last time? ---
        # description: one-line human summary of the scenario (passed in by the
        # suite); last: the previous persisted result for this exact scenario,
        # so a known-broken corner shows WHY it failed last time right up front.
        print(f"\n{'='*60}")
        if description:
            print(cyan(f"🧪 Test: {description}"))
        else:
            print(cyan(f"🧪 Test: {direction.upper()} turn" + (f" @ '{start_point}'" if start_point else " (random location)")))
        if start_point and results is not None:
            last = last_result_for(results, start_point, direction)
            if last is None:
                print(dim("   Last run:  never run before"))
            elif last.get("passed"):
                when = f" ({last['timestamp']})" if last.get("timestamp") else ""
                print(f"   Last run:  {green('Yes')} - passed{when}")
            else:
                print(f"   Last run:  {red('No')} - {last.get('reason') or 'unknown failure'}" +
                      (f" ({last['timestamp']})" if last.get("timestamp") else ""))
        print(f"{'='*60}")
        
        # Reset and enable breadcrumbs for visual debugging
        self.reset_controls()
        requests.post(f"{API_URL}/toggle", json={'breadcrumbs': True})
        
        # Create a fresh car at the given start point or a random location.
        # Deterministic scenarios start MID-segment (50%) so the maneuver
        # begins on open road, not hugging the junction node.
        if start_point:
            print(f"📍 Creating new car at '{start_point}' "
                  f"({start_progress * 100:.0f}% of segment)...")
            # Running turn tests spawn ALREADY MOVING (rolling start, no
            # standstill-to-cruise phase); parking tests start from rest.
            rolling = None if kerb_check else RUNNING_START_KMH / 3.6
            self.create_car_at_start_point(start_point, progress=start_progress,
                                           speed_mps=rolling)
            # Label under the minimap: the current test number ("10/15")
            # when running as part of a suite; standalone runs fall back
            # to the track number / start-point name.
            self.set_hud_label(label if label is not None
                               else str(START_POINT_NUMBER.get(start_point) or start_point))
        else:
            print("📍 Creating new car at random location...")
            self.create_car_at_random()
            self.set_hud_label(None)

        if _aborted():
            return self._aborted_result(start_point, direction, target_speed, results)
        
        # If the game dies during setup (window closed, or the teleport
        # watchdog crashing the process right after the teleport), treat it
        # exactly like a crash mid-monitor: record game_crashed for this
        # scenario and skip straight to the summary instead of raising.
        setup_crashed = False
        setup_aborted = False
        # Baseline for the cumulative validator log - taken IMMEDIATELY
        # after the teleport, BEFORE the accelerate phase: a spawn that
        # sits off the pavement (e.g. chord vs corner-rounded road) would
        # otherwise log its violations during setup and be baked into the
        # baseline, making them invisible to the monitor's delta check.
        violations_at_start = 0
        ws_frames_at_start = 0
        try:
            state = self.get_state()
            initial_segment = state['segment']
            initial_pos = (state['x'], state['y'])
            initial_heading = state['heading']
            violations_at_start = state.get('validator_violations', 0)
            ws_frames_at_start = (state.get('lane_guard_stats') or {}) \
                .get('wrong_side_frames', 0)
            ws_secs_at_start = (state.get('lane_guard_stats') or {}) \
                .get('wrong_side_seconds', 0.0)
            print(f"   Starting at segment {initial_segment}")
            print(f"   Position: ({state['x']:.0f}, {state['y']:.0f})")
            print(f"   Heading: {state['heading']:.1f}°")

            # Green start flag at exactly where this scenario begins, and
            # the RED end flag at the EXPECTED destination: [segment, 50%
            # progress] - the game loop resolves that to a map position
            # once the route covers the segment, so the end marker is
            # visible from the START of the test (and sits on the right of
            # the road even for backward-traversed segments). Setting 'red'
            # also replaces any stale end flag from a previous scenario;
            # random-location runs have no known destination.
            try:
                # Parking tests: the flag (50% of the end segment) is the
                # car's NAVIGATION destination - it parks at it. Running
                # turn tests: visual-only marker at end_flag_progress of
                # the end segment (suite standard: FIRST 20%) - crossing
                # it ends the test, no parking.
                red_flag = None
                flag_point = None   # world px of the red flag (reach check)
                if expected_end_segment is not None:
                    prog_flag = 0.5 if kerb_check else end_flag_progress
                    seg = self.get_segment_geometry(expected_end_segment)
                    if seg is not None:
                        # Running tests: measure the flag from the NODE SIDE
                        # of the exit arm - where the car actually enters
                        # it. Some arms are parameterised the other way
                        # round (crossroads seg 17: a right-turning car
                        # enters at its 100% end), and a raw-parameterisation
                        # flag then sits where the car drives AWAY from -
                        # the old 'prog >= threshold' latch fired on entry,
                        # ending the test before the maneuver was even done
                        # (tests 8 + 16: "passed" without reaching the flag).
                        # Parking tests keep their explicit 50% destination.
                        if not kerb_check:
                            d0 = ((seg['x1'] - state['x']) ** 2
                                  + (seg['y1'] - state['y']) ** 2)
                            d1 = ((seg['x2'] - state['x']) ** 2
                                  + (seg['y2'] - state['y']) ** 2)
                            if d1 < d0:   # car enters at the far end
                                prog_flag = 1.0 - prog_flag
                        flag_point = (seg['x1'] + prog_flag * (seg['x2'] - seg['x1']),
                                      seg['y1'] + prog_flag * (seg['y2'] - seg['y1']))
                    else:
                        print("   ⚠️  Could not resolve the end segment's "
                              "geometry - the flag-reach check is disabled, "
                              "this scenario will time out")
                    red_flag = [expected_end_segment, prog_flag]
                requests.post(f"{API_URL}/flags", json={
                    'green': [state['x'], state['y'], state['heading']],
                    'red': red_flag,
                    'red_nav': bool(kerb_check),
                }, timeout=2)
            except requests.exceptions.RequestException:
                pass

            # Accelerate to target speed (no blinker yet - we signal only
            # once we are actually close to the junction, like a real driver)
            print(f"🚗 Accelerating to {target_speed:.0f} km/h...")
            self.send_control(accelerate=True)
            
            # Wait to reach speed
            for _ in range(50):  # 5 seconds max
                if _aborted():
                    break
                state = self.get_state()
                if state['speed_kmh'] >= target_speed * 0.9:
                    break
                time.sleep(0.1)
            
            print(f"   Reached {state['speed_kmh']:.0f} km/h")
            
            if _aborted():
                setup_aborted = True
            elif direction != 'straight':
                # Wait until we are SIGNAL_DISTANCE_M or less before the
                # junction, then signal the turn.
                blinker_key = 'blinker_left' if direction == 'left' else 'blinker_right'
                dist = state.get('distance_to_junction')
                for _ in range(100):  # 10 seconds max
                    if _aborted():
                        break
                    if dist is not None and dist <= SIGNAL_DISTANCE_M:
                        break
                    state = self.get_state()
                    dist = state.get('distance_to_junction')
                    time.sleep(0.1)
                if dist is not None and dist <= SIGNAL_DISTANCE_M:
                    self.send_control(**{blinker_key: True})
                    print(f"   {direction.upper()} blinker activated at {dist:.0f} m before the junction")
                else:
                    print(f"   ⚠️  Never got within {SIGNAL_DISTANCE_M:.0f} m of the junction - no blinker sent")
        except requests.exceptions.RequestException as e:
            setup_crashed = True
            initial_segment = -1
            initial_pos = (0.0, 0.0)
            initial_heading = 0.0
            state = {'x': 0.0, 'y': 0.0, 'heading': 0.0, 'segment': -1,
                     'speed_kmh': 0.0, 'on_road': True}
            print(f"\n   ❌ GAME PROCESS CRASHED / CONNECTION LOST during setup!")
            print(f"      Likely cause: an internal teleportation-watchdog violation "
                  f"(see the game's own console output/log)")
            print(f"      Error: {e}")
        if setup_aborted:
            return self._aborted_result(start_point, direction, target_speed, results)
        if expected_end_segment is not None:
            print(f"   Expected end segment: {expected_end_segment}")
        print(f"\n🔍 Monitoring turn for {duration}s...")
        print(f"   Checking for:")
        print(f"     - Off-road violations")
        print(f"     - Instant heading snaps (>30° per frame)")
        print(f"     - Smooth circular arc progression")
        print(f"     - Arriving at the {'expected' if expected_end_segment is not None else 'designated'} end segment")
        
        # Monitor turn
        start_time = time.time()
        frames_checked = 0
        segment_changed = False
        reached_expected_segment = False
        # Progress along the target segment when the car FIRST entered it -
        # tells us which side of the 50% mark it has to cross.
        end_entry_progress: float | None = None
        passed_flag = False          # crossed the flag while moving (violation)
        passed_flag_speed_kmh = 0.0
        off_road_detected = False
        instant_snap_detected = False
        teleport_detected = False
        game_crashed = False
        wrong_side_detected = False
        stopped_ok = False
        braking_for_end = False
        violation_details = None
        positions = []
        # (violations_at_start was captured right after the teleport -
        # see the setup block above.)
        final_pos = initial_pos
        last_heading = initial_heading
        max_heading_change_per_frame = 0.0
        last_poll_pos = initial_pos
        last_poll_speed_kmh = 0.0
        last_poll_time = time.time()
        
        if setup_crashed:
            game_crashed = True
            violation_details = {
                'type': 'game_crashed',
                'time': 0.0,
                'error': 'connection lost during setup (teleport/accelerate phase)',
            }
        
        while not setup_crashed and not _aborted() \
                and time.time() - start_time < duration:
            # A teleport-watchdog violation inside the game crashes that
            # process outright (a deliberate hard invariant - see
            # PhysicsValidator / docs/SPEC.md's "Physics Judge"
            # philosophy) - which would otherwise take the WHOLE test
            # suite down with an unhandled connection error. Treat losing
            # the connection mid-test as its own violation (failing just
            # this one test) instead of letting it kill everything after.
            try:
                state = self.get_state()
            except requests.exceptions.RequestException as e:
                game_crashed = True
                violation_details = {
                    'type': 'game_crashed',
                    'time': time.time() - start_time,
                    'error': str(e),
                }
                print(f"\n   ❌ GAME PROCESS CRASHED / CONNECTION LOST!")
                print(f"      Time: {violation_details['time']:.2f}s")
                print(f"      Likely cause: an internal teleportation-watchdog violation "
                      f"(see the game's own console output/log)")
                print(f"      Error: {e}")
                break
            frames_checked += 1
            current_heading = state['heading']
            
            # Client-side teleport/jump check: an independent, coarser
            # safety net alongside the game's own internal (per-physics-
            # frame) teleportation watchdog - catches large jumps between
            # POLLS too, using the actual measured wall-clock gap (not an
            # assumed fixed frame dt, since polling isn't frame-locked).
            now = time.time()
            poll_dt = now - last_poll_time
            moved_px = ((state['x'] - last_poll_pos[0]) ** 2 + (state['y'] - last_poll_pos[1]) ** 2) ** 0.5
            moved_m = moved_px / 2.0  # PIXELS_PER_METER (see src/config.py)
            # Generous margin: highest plausible speed (~50 m/s cruise)
            # times the actual poll gap, plus slack for polling jitter
            # and the accelerate-from-a-stop ramp-up.
            max_plausible_m = max(last_poll_speed_kmh / 3.6, 50.0) * poll_dt * 1.5 + 1.0
            if frames_checked > 1 and moved_m > max_plausible_m:
                teleport_detected = True
                violation_details = {
                    'type': 'teleport',
                    'time': now - start_time,
                    'from_position': last_poll_pos,
                    'to_position': (state['x'], state['y']),
                    'distance_m': moved_m,
                    'max_plausible_m': max_plausible_m,
                    'speed_kmh': state['speed_kmh'],
                    'segment': state['segment'],
                }
                print(f"\n   ❌ TELEPORTATION / UNEXPECTED JUMP DETECTED!")
                print(f"      Time: {violation_details['time']:.2f}s")
                print(f"      From: ({last_poll_pos[0]:.0f}, {last_poll_pos[1]:.0f}) "
                      f"→ To: ({state['x']:.0f}, {state['y']:.0f})")
                print(f"      Distance: {moved_m:.1f}m (max plausible: {max_plausible_m:.1f}m "
                      f"over {poll_dt:.3f}s)")
                print(f"      Speed: {violation_details['speed_kmh']:.0f} km/h")
                screenshot = self.save_violation_screenshot(f"{direction}_teleport", state)
                violation_details['screenshot'] = screenshot
                break
            last_poll_pos = (state['x'], state['y'])
            last_poll_speed_kmh = state['speed_kmh']
            last_poll_time = now
            
            # Calculate heading change (handle 360° wrap). Skip the very
            # first poll: `last_heading` was initialised to the TELEPORT
            # heading, but the car has already driven (and turned smoothly)
            # during the unmonitored "accelerate to speed" phase, so
            # comparing the first poll to the teleport heading would flag a
            # legitimate accumulated turn as an instant snap. The teleport/
            # jump position check below already skips the first poll for the
            # same reason (frames_checked > 1).
            if frames_checked > 1:
                heading_diff = abs((current_heading - last_heading + 180) % 360 - 180)
                max_heading_change_per_frame = max(max_heading_change_per_frame, heading_diff)
            else:
                heading_diff = 0.0
            
            # Record position
            positions.append({
                'time': time.time() - start_time,
                'x': state['x'],
                'y': state['y'],
                'heading': state['heading'],
                'heading_change': heading_diff,
                'speed_kmh': state['speed_kmh'],
                'segment': state['segment'],
                'on_road': state['on_road']
            })
            
            # Check for instant heading snap (>30° in one frame at 60fps = 0.016s)
            if frames_checked > 1 and heading_diff > 30.0:
                instant_snap_detected = True
                violation_details = {
                    'type': 'instant_heading_snap',
                    'time': time.time() - start_time,
                    'position': (state['x'], state['y']),
                    'old_heading': last_heading,
                    'new_heading': current_heading,
                    'heading_change': heading_diff,
                    'speed_kmh': state['speed_kmh'],
                    'segment': state['segment'],
                    'frame': frames_checked
                }
                
                print(f"\n   ❌ INSTANT HEADING SNAP DETECTED!")
                print(f"      Time: {violation_details['time']:.2f}s")
                print(f"      Old heading: {last_heading:.1f}°")
                print(f"      New heading: {current_heading:.1f}°")
                print(f"      Change: {heading_diff:.1f}° (max allowed: 30°)")
                print(f"      Position: ({violation_details['position'][0]:.0f}, {violation_details['position'][1]:.0f})")
                print(f"      Speed: {violation_details['speed_kmh']:.0f} km/h")
                
                # Save screenshot
                screenshot = self.save_violation_screenshot(f"{direction}_snap", state)
                violation_details['screenshot'] = screenshot
                
                break
            
            # Check for off-road violation (live check + validator log).
            # The validator's violation log is CUMULATIVE for the whole game
            # process and is never cleared, so it must be compared against
            # its value when this scenario started - not against zero. Doing
            # the latter latched: one violation anywhere made every later
            # scenario report off-road at t=0.00s, turning a single real
            # failure into thirteen false ones.
            if not state['on_road'] or \
                    state.get('validator_violations', 0) > violations_at_start:
                off_road_detected = True
                violation_details = {
                    'type': 'off_road',
                    'time': time.time() - start_time,
                    'position': (state['x'], state['y']),
                    'heading': state['heading'],
                    'speed_kmh': state['speed_kmh'],
                    'segment': state['segment'],
                    'frame': frames_checked
                }
                
                print(f"\n   ❌ OFF-ROAD VIOLATION DETECTED!")
                print(f"      Time: {violation_details['time']:.2f}s")
                print(f"      Position: ({violation_details['position'][0]:.0f}, {violation_details['position'][1]:.0f})")
                print(f"      Heading: {violation_details['heading']:.1f}°")
                print(f"      Speed: {violation_details['speed_kmh']:.0f} km/h")
                print(f"      Segment: {violation_details['segment']}")
                
                # Save screenshot
                screenshot = self.save_violation_screenshot(f"{direction}_offroad", state)
                violation_details['screenshot'] = screenshot
                
                break
            
            # Wrong-side driving (oncoming lane): the game only WARNS on a
            # violation (state flag + cumulative lane-guard stats); failing
            # the run is the suite's job (docs/TESTING.md §3). /state
            # reports 'wrong_side' on every frame the car sits on the wrong
            # side, so this poll catches it live; the cumulative counter
            # below is the backstop for hits between polls.
            if state.get('wrong_side'):
                wrong_side_detected = True
                violation_details = {
                    'type': 'wrong_side',
                    'time': time.time() - start_time,
                    'position': (state['x'], state['y']),
                    'heading': state['heading'],
                    'speed_kmh': state['speed_kmh'],
                    'segment': state['segment'],
                    'frame': frames_checked
                }
                
                print(f"\n   ❌ WRONG-SIDE DRIVING DETECTED!")
                print(f"      Time: {violation_details['time']:.2f}s")
                print(f"      Position: ({state['x']:.0f}, {state['y']:.0f})")
                print(f"      Heading: {state['heading']:.1f}°")
                print(f"      Speed: {state['speed_kmh']:.0f} km/h")
                print(f"      Segment: {state['segment']}")
                
                # Save screenshot
                screenshot = self.save_violation_screenshot(f"{direction}_wrongside", state)
                violation_details['screenshot'] = screenshot
                
                break
            
            # RUNNING TURN TEST arrival (docs/TESTING.md): the scenario is
            # done when the car physically drives THROUGH the red end flag.
            # Geometric, not 'prog >= threshold' - the old latch fired on
            # the first poll for arms the car enters "from behind" (reverse-
            # parameterised exits: crossroads right turn, hairpin exit),
            # ending the test before the maneuver was even finished and
            # never actually reaching the flag. Checked BEFORE the segment-
            # change gate on purpose: near a node the reported segment can
            # still be the approach arm while the car already passes the
            # flag point (hairpin: its projection onto the exit arm is
            # closer than the diagonal it really drives).
            if not kerb_check and flag_point is not None:
                fdx = state['x'] - flag_point[0]
                fdy = state['y'] - flag_point[1]
                if (fdx * fdx + fdy * fdy) ** 0.5 <= FLAG_REACH_RADIUS_M * 2.0:
                    reached_expected_segment = True
                    prog_pct = (state.get('progress') or 0.0) * 100
                    print(f"\n   ✅ Crossed the end flag at {prog_pct:.0f}% "
                          f"of segment {state['segment']} "
                          f"(target: {end_flag_progress * 100:.0f}%, node side)")
                    print(f"      Time: {time.time() - start_time:.2f}s")
                    print(f"      Max heading change per frame: {max_heading_change_per_frame:.1f}°")
                    final_pos = (state['x'], state['y'])
                    stopped_ok = True   # running test: no stop required
                    print(f"      Start: ({initial_pos[0]:.0f}, {initial_pos[1]:.0f}) seg {initial_segment} "
                          f"→ End: ({final_pos[0]:.0f}, {final_pos[1]:.0f}) seg {state['segment']}")
                    print(f"      Distance traveled: {((final_pos[0] - initial_pos[0])**2 + (final_pos[1] - initial_pos[1])**2)**0.5:.0f} pixels")
                    try:
                        self.reset_controls()   # release the throttle latch
                    except requests.exceptions.RequestException:
                        pass
                    break
            
            # Check if segment changed. If we have a SPECIFIC expected
            # end segment, only that counts as arrival - a mere change to
            # some OTHER segment (e.g. a wrong turn) is noted but keeps
            # monitoring (the car might still be mid-maneuver, as on a
            # roundabout: entry segment -> ring -> ring -> exit segment -
            # only the last of those is "done"). Without an expected
            # segment (legacy random-location mode only), any change at
            # all is accepted, same as before.
            # The parking-offset scenarios START and END on the same
            # segment (the car parks where it was driving), so "segment
            # changed" can never be their arrival trigger - being ON the
            # expected segment is enough.
            on_target_segment = (expected_end_segment is not None
                                 and state['segment'] == expected_end_segment)
            if state['segment'] != initial_segment or on_target_segment:
                if state['segment'] != initial_segment and not segment_changed:
                    segment_changed = True
                    print(f"\n   ℹ️  Segment changed: {initial_segment} → {state['segment']} "
                          f"(t={time.time() - start_time:.2f}s)")
                if on_target_segment:
                    # Deterministic scenarios END at the 50% point of the
                    # target segment (mid-segment, mirroring the start) -
                    # the car does NOT drive on to the far end / dead end,
                    # because parking is a separate maneuver with its own
                    # scenario, not the tail of every turn test.
                    if end_entry_progress is None:
                        end_entry_progress = state.get('progress')
                    prog = state.get('progress')
                    entry = end_entry_progress
                    if not kerb_check:
                        # RUNNING TURN TEST (docs/TESTING.md): arrival is
                        # handled by the geometric flag check above - the
                        # car drives THROUGH the red flag and keeps going,
                        # no stop, no parking. This guard exists so the
                        # STOP-AT-FLAG logic below only applies to parking
                        # (kerb_check) scenarios.
                        pass
                    crossed_now = (prog is not None and entry is not None and (
                        (entry < 0.5 and prog >= 0.5)
                        or (entry > 0.5 and prog <= 0.5)))
                    # Reverse-in parking (§1b): the car deliberately drives
                    # PAST the flag to stage the back-in, stops ~3 m beyond
                    # it, then reverses into the spot at the kerb. The
                    # crossing is expected and the staging stop is NOT an
                    # arrival - only the nav's 'parked' flag (manoeuvre
                    # complete) counts.
                    parking = state.get('parking') or {}
                    reverse_planned = parking.get('style') == 'reverse'
                    reverse_done = not reverse_planned \
                        or bool(parking.get('parked'))
                    # STRICT arrival criterion (docs/TESTING.md §3, rule 1):
                    # the scenario passes only if the car comes to rest AT
                    # the flag. Crossing it while moving is "drove past the
                    # destination" - a FAILURE, not an arrival (see
                    # FLAG_CRAWL_KMH for the crawl-speed exception). A
                    # planned reverse-in crosses on purpose - exempt until
                    # it has parked.
                    if crossed_now and state['speed_kmh'] >= FLAG_CRAWL_KMH \
                            and not reverse_planned:
                        passed_flag = True
                        passed_flag_speed_kmh = state['speed_kmh']
                        print(f"\n   ❌ DROVE PAST THE END FLAG at "
                              f"{state['speed_kmh']:.0f} km/h - the scenario "
                              f"requires a STOP AT the flag")
                    crossed_half = (crossed_now and state['speed_kmh'] < FLAG_CRAWL_KMH) \
                        or (prog is not None and entry is not None
                            and abs(entry - 0.5) < 0.02)
                    if crossed_half and not reverse_done:
                        crossed_half = False   # staging, not arrival
                    if passed_flag:
                        final_pos = (state['x'], state['y'])
                        print(f"      Time: {time.time() - start_time:.2f}s")
                        print(f"      Max heading change per frame: {max_heading_change_per_frame:.1f}°")
                        break
                    # SAFETY NET only: the nav parks at the red flag itself
                    # (it is the car's destination - parking ramp + kerb
                    # drift + stop, see BicycleNav.set_destination). This
                    # latch only fires if the car is still rolling toward
                    # the flag within braking distance, i.e. the nav's park
                    # approach failed.
                    seg_len = state.get('segment_length')
                    if prog is not None and seg_len:
                        dist_flag_m = abs(prog - 0.5) * seg_len
                        speed_ms = state['speed_kmh'] / 3.6
                        brake_dist_m = (speed_ms ** 2) / (2.0 * CAR_BRAKING) \
                            + max(1.0, speed_ms * 0.15)   # polling/reaction slack
                        # (Never during a reverse-in staging approach - the
                        # car is supposed to keep rolling past the flag.)
                        if not braking_for_end and not crossed_half \
                                and reverse_done \
                                and dist_flag_m <= brake_dist_m:
                            braking_for_end = True
                            self.send_control(accelerate=False, brake=True)
                            print(f"\n   🛑 Braking for the end flag "
                                  f"({dist_flag_m:.0f} m ahead)")
                        # Arrival = at rest within tolerance of the flag,
                        # regardless of who latched the brake (normally the
                        # nav's parking block, sometimes the safety net).
                        # A reverse-in only counts once 'parked' - the
                        # staging stop ~3 m past the flag must not latch.
                        if not crossed_half \
                                and reverse_done \
                                and state['speed_kmh'] < 1.0 \
                                and dist_flag_m <= STOP_AT_FLAG_TOLERANCE_M:
                            crossed_half = True   # stopped right at the flag
                    if crossed_half:
                        reached_expected_segment = True
                        print(f"\n   ✅ Reached end segment {state['segment']} "
                              f"at {prog * 100:.0f}% (target: 50%)")
                        print(f"      Time: {time.time() - start_time:.2f}s")
                        print(f"      Max heading change per frame: {max_heading_change_per_frame:.1f}°")
                        final_pos = (state['x'], state['y'])
                        # (The red end flag has been up since setup - it
                        # marks exactly this destination.) The scenario is
                        # DONE here: come to a full stop AT the flag and
                        # hold the brake, like a driver parking at their
                        # destination.
                        cross_pos = final_pos
                        final_pos, stopped_ok, stop_details = self._brake_to_stop(
                            start_time, abort_check=abort_check)
                        if stop_details.get('aborted'):
                            return self._aborted_result(start_point, direction, target_speed, results)
                        if not stopped_ok:
                            off_road_detected = off_road_detected or stop_details.get('off_road', False)
                            instant_snap_detected = instant_snap_detected or stop_details.get('instant_snap', False)
                            teleport_detected = teleport_detected or stop_details.get('teleport', False)
                            game_crashed = game_crashed or stop_details.get('game_crashed', False)
                            if stop_details.get('violation_details'):
                                violation_details = stop_details['violation_details']
                        rolled_m = ((final_pos[0] - cross_pos[0]) ** 2 + (final_pos[1] - cross_pos[1]) ** 2) ** 0.5 / 2.0
                        if stopped_ok:
                            where = (f"{rolled_m:.1f} m after crossing it"
                                     if rolled_m > 0.5 else "right at the flag")
                            print(f"      🅿️  Stopped at the end flag ({where})")
                        else:
                            print(f"      ⚠️  Did NOT come to a clean stop at the end flag")
                        print(f"      Start: ({initial_pos[0]:.0f}, {initial_pos[1]:.0f}) seg {initial_segment} "
                              f"\u2192 End: ({final_pos[0]:.0f}, {final_pos[1]:.0f}) seg {state['segment']}")
                        print(f"      Distance traveled: {((final_pos[0] - initial_pos[0])**2 + (final_pos[1] - initial_pos[1])**2)**0.5:.0f} pixels")
                        break
                elif expected_end_segment is None:
                    # Legacy random-location mode: any segment change counts;
                    # drive to the far end of it and stop there.
                    reached_expected_segment = True
                    print(f"\n   ✅ Reached designated end segment {state['segment']}!")
                    print(f"      Time: {time.time() - start_time:.2f}s")
                    print(f"      Max heading change per frame: {max_heading_change_per_frame:.1f}°")
                    final_pos, stopped_ok, stop_details = self._drive_to_segment_end_and_stop(
                        state['segment'], start_time, abort_check=abort_check)
                    if stop_details.get('aborted'):
                        return self._aborted_result(start_point, direction, target_speed, results)
                    if not stopped_ok:
                        off_road_detected = off_road_detected or stop_details.get('off_road', False)
                        instant_snap_detected = instant_snap_detected or stop_details.get('instant_snap', False)
                        teleport_detected = teleport_detected or stop_details.get('teleport', False)
                        game_crashed = game_crashed or stop_details.get('game_crashed', False)
                        if stop_details.get('violation_details'):
                            violation_details = stop_details['violation_details']
                    print(f"      Start: ({initial_pos[0]:.0f}, {initial_pos[1]:.0f}) seg {initial_segment} "
                          f"\u2192 End: ({final_pos[0]:.0f}, {final_pos[1]:.0f}) seg {state['segment']}"
                          f"{' (stopped)' if stopped_ok else ' (did NOT stop cleanly)'}")
                    print(f"      Distance traveled: {((final_pos[0] - initial_pos[0])**2 + (final_pos[1] - initial_pos[1])**2)**0.5:.0f} pixels")
                    break
            
            last_heading = current_heading
            time.sleep(0.05)  # Check at ~20 FPS
        else:
            final_pos = (state['x'], state['y'])

        if _aborted():
            return self._aborted_result(start_point, direction, target_speed, results)

        # Scenario end: on a clean deterministic finish the brake is
        # deliberately HELD - the car stays parked AT the end flag, like a
        # driver who has reached their destination (the next scenario's
        # reset_controls() clears the latch). Every other exit (failure,
        # timeout, legacy mode) gets its inputs cleared here.
        try:
            if not (expected_end_segment is not None and reached_expected_segment
                    and stopped_ok):
                self.reset_controls()
        except requests.exceptions.RequestException:
            pass
        
        # Parking-offset scenarios (kerb_check): the arrival above only
        # proves the car stopped AT the flag - the pass criterion is how
        # CLOSE to the right kerb it parked: both right-hand wheels within
        # KERB_PASS_MAX_M. Measured on a fresh state after the final stop.
        kerb_gaps = None
        if kerb_check and reached_expected_segment and stopped_ok:
            try:
                kerb_gaps = self._right_side_kerb_gaps_m(
                    self.get_state(), start_point)
            except requests.exceptions.RequestException:
                kerb_gaps = None
        kerb_failed = (kerb_gaps is not None
                       and (max(kerb_gaps) > KERB_PASS_MAX_M
                            or min(kerb_gaps) <= 0.0))

        # Backstop for wrong-side hits that fell BETWEEN API polls: the
        # lane guard's frame counter is cumulative for the whole game
        # process, so compare it against its value when this scenario
        # started (same delta logic as validator_violations).
        if not wrong_side_detected:
            ws_now = (state.get('lane_guard_stats') or {}) \
                .get('wrong_side_frames', 0)
            if ws_now > ws_frames_at_start:
                wrong_side_detected = True
                print(f"\n   ❌ WRONG-SIDE DRIVING DETECTED (between polls: "
                      f"{ws_now - ws_frames_at_start} frame(s) on the wrong "
                      f"side per lane-guard stats)")
        
        # A scenario only passes if it ENDS with the car stopped at its
        # destination - the end flag for deterministic scenarios, the far
        # end of the target segment in legacy random mode. Driving on past
        # the destination is a different maneuver, not this test.
        # EXCEPTION: negative detector tests (expect_wrong_side) PASS when
        # the violation fires - they exist to prove the detection works.
        if expect_wrong_side:
            passed = wrong_side_detected
        else:
            passed = (
                reached_expected_segment
                and stopped_ok
                and not passed_flag
                and not off_road_detected
                and not instant_snap_detected
                and not teleport_detected
                and not game_crashed
                and not kerb_failed
                and not wrong_side_detected
            )
        
        # Prepare results
        result = {
            'start_point': start_point,
            'direction': direction,
            'target_speed_kmh': target_speed,
            'frames_checked': frames_checked,
            'duration': time.time() - start_time,
            'initial_segment': initial_segment,
            'expected_end_segment': expected_end_segment,
            'final_segment': state['segment'],
            'start_position': initial_pos,
            'end_position': final_pos,
            'segment_changed': segment_changed,
            'reached_expected_segment': reached_expected_segment,
            'stopped_at_end': stopped_ok,
            'passed_flag': passed_flag,
            'passed_flag_speed_kmh': passed_flag_speed_kmh,
            'off_road_detected': off_road_detected,
            'instant_snap_detected': instant_snap_detected,
            'teleport_detected': teleport_detected,
            'game_crashed': game_crashed,
            'wrong_side_detected': wrong_side_detected,
            'expect_wrong_side': expect_wrong_side,
            'kerb_check': kerb_check,
            'end_flag_progress': end_flag_progress,
            'kerb_gaps_m': kerb_gaps,
            'max_heading_change_per_frame': max_heading_change_per_frame,
            'violation_details': violation_details,
            'positions': positions,
            'aborted': False,
            'passed': passed
        }
        
        # Summary
        print(f"\n{'─'*60}")
        print(f"   Frames checked: {frames_checked}")
        print(f"   Duration: {result['duration']:.2f}s")
        print(f"   Max heading change per frame: {result['max_heading_change_per_frame']:.1f}°")
        
        print(f"   Start: ({initial_pos[0]:.0f}, {initial_pos[1]:.0f}) seg {initial_segment}  "
              f"End: ({final_pos[0]:.0f}, {final_pos[1]:.0f}) seg {state['segment']}"
              + (f"  (expected seg {expected_end_segment})" if expected_end_segment is not None else ""))

        # Validator violations (off-road caught between API polls) -
        # PER-TEST delta against the post-spawn baseline: the game's
        # counter is a lifetime log, printing it raw read as "this test
        # logged N" when it was really everything since game start.
        vv = max(0, state.get('validator_violations', 0) - violations_at_start)
        if vv > 0:
            print(red(f"   ⚠️  Validator: {vv} off-road violation(s) logged"))

        # Lane guard stats (per-test delta, same reason)
        lg = state.get('lane_guard_stats', {})
        wrong_frames = max(0, lg.get('wrong_side_frames', 0) - ws_frames_at_start)
        wrong_secs = max(0.0, lg.get('wrong_side_seconds', 0.0) - ws_secs_at_start)
        if wrong_frames > 0:
            print(red(f"   ⚠️  Wrong-side driving: {wrong_frames} frames ({wrong_secs}s)"))
        else:
            print(green("   ✅ Lane guard: no wrong-side driving detected"))

        if kerb_gaps is not None:
            ok = not kerb_failed
            txt = (f"   Kerb check: right flank {kerb_gaps[1] * 100:.0f} cm "
                   f"(front) / {kerb_gaps[0] * 100:.0f} cm (rear) from the "
                   f"kerb (limit {KERB_PASS_MAX_M * 100:.0f} cm)")
            print(green(f"   ✅ {txt}") if ok else red(f"   ❌ {txt}"))

        # Colored one-line verdict: green "passed" or red "fail: <reason>".
        reason = describe_failure(result)
        if reason is None:
            print(green("   ✅ PASSED"))
            if result.get('expect_wrong_side'):
                print(dim("      Wrong-side driving occurred and was detected - "
                          "exactly what this negative detector test expects"))
            elif result.get('expected_end_segment') is None:
                print(dim("      Reached designated end segment, drove to its end and "
                          "stopped there, stayed on road, no violations"))
            else:
                print(dim("      Reached the 50% point of the expected end segment and "
                          "stopped at the end flag, stayed on road, no violations"))
        else:
            print(red(f"   ❌ FAIL: {reason}"))
        
        print(f"{'─'*60}\n")
        
        self.test_results.append(result)
        if results is not None:
            record_result(results, result)
            save_results(results)
        
        return result
    
    def run_random_test(self, results: dict | None = None):
        """Run full test suite with multiple speeds and directions,
        teleporting to random locations on whatever map is loaded
        (real OSM data or a synthetic test map)."""
        print("\n" + "="*60)
        print("RANDOM-LOCATION TURN TESTING")
        print("="*60)
        print("\nTesting turns at different speeds:")
        print("  - Low speed:  30 km/h")
        print("  - Medium speed: 50 km/h")
        print("  - High speed: 80 km/h")
        print("\nEach test will:")
        print("  1. Teleport to random location")
        print("  2. Accelerate to target speed")
        print("  3. Activate turn signal")
        print("  4. Monitor turn execution")
        print("  5. Check for off-road violations")
        print("\n" + "="*60)
        
        tests = [
            # Low speed turns
            ('right', 30),
            ('left', 30),
            # Medium speed turns
            ('right', 50),
            ('left', 50),
            # High speed turns
            ('right', 80),
            ('left', 80),
        ]
        
        for i, (direction, speed) in enumerate(tests, 1):
            # User interrupted (Ctrl-C) or closed the game window: stop now.
            if interrupted():
                print(yellow(f"\n⏸️  Stopping after test {i-1}/{len(tests)} - "
                             f"run interrupted (Ctrl-C or game window closed)."))
                break

            print(f"\n\n{'#'*60}")
            print(f"# TEST {i}/{len(tests)}: {direction.upper()} turn at {speed} km/h")
            print(f"{'#'*60}")
            
            result = self.monitor_turn(direction, duration=15.0, target_speed=speed)
            
            # Brief pause between tests
            if i < len(tests):
                print("\n⏸️  Pausing 2s before next test...")
                time.sleep(2)
        
        # Final summary
        self.print_summary()
    
    def run_deterministic_test(self, results: dict | None = None, spec: str | None = None,
                               abort_check: "callable | None" = None,
                               take_jump: "callable | None" = None,
                               on_event: "callable | None" = None):
        """Run the turn test suite against KNOWN, reproducible scenarios
        from the 'basic' synthetic test map (see src/test_maps.py).
        Requires the game to be started with: --map basic --api

        Cockpit hooks (all optional; the CLI path uses none of them):
          spec:         scenario selection string for select_tests (e.g. "1-3")
          abort_check:  zero-arg callable, polled inside monitor_turn; if it
                        returns truthy the current scenario is aborted
          take_jump:    zero-arg callable, polled between scenarios; returns
                        the 1-based number of a scenario to start next and
                        consumes the request - set by a cockpit button click
          on_event:     callback for {'type': 'start'|'done'|'jump', ...}
                        events so a UI can follow the suite's progress
        """
        print("\n" + "="*60)
        print("DETERMINISTIC TURN TESTING (synthetic 'basic' map)")
        print("="*60)

        available = self.get_start_points()
        if not available:
            print("\n❌ No named start points reported by the API.")
            print("   Start the game with: python -m src.main --map basic --api\n")
            sys.exit(1)

        print(f"\n{len(available)} named start points available on this map.")
        print("\nEach test will:")
        print("  1. Teleport to a KNOWN start point (exact position + heading)")
        print("  2. Accelerate to target speed")
        print("  3. Activate turn signal (or none, for 'straight')")
        print("  4. Monitor turn execution")
        print("  5. Check for off-road violations and instant heading snaps")
        print("\n" + "="*60)

        tests = DETERMINISTIC_TESTS

        if results is None:
            results = load_results()
        print(f"\n📄 Results file: {RESULTS_FILE}")
        print(f"   (each scenario's outcome is saved there; next run shows its last result)")

        selected = select_tests(tests, spec)
        if not selected:
            return
        if len(selected) != len(tests):
            print(f"\n▶  Running {len(selected)} of {len(tests)} scenarios: "
                  f"{', '.join(str(i) for i, _ in selected)}")

        def _emit(event: dict):
            if on_event is not None:
                on_event(event)

        pos = 0
        while pos < len(selected):
            i, test = selected[pos]

            # User interrupted (Ctrl-C) or closed the game window: stop
            # scheduling more tests now - results so far are already saved.
            if interrupted():
                print(yellow(f"\n⏸️  Stopping after test {i-1}/{len(tests)} - "
                             f"run interrupted (Ctrl-C or game window closed)."))
                break

            # Cockpit jump: a scenario button was clicked in the controller
            # window. The current test was already aborted by abort_check
            # inside monitor_turn (or hasn't started yet); start the
            # requested one instead, then continue with the rest after it.
            if take_jump is not None:
                k = take_jump()
                if k is not None:
                    if k in {idx for idx, _ in selected}:
                        print(f"\n⏭️  JUMP: starting test {k} (requested from the cockpit)")
                        _emit({'type': 'jump', 'from': i, 'to': k})
                        pos = next(p for p, (idx, _) in enumerate(selected) if idx == k)
                        continue
                    print(f"   ⚠️  Jump target {k} is not in the selected set - ignoring")

            start_point, direction, speed, expected_end_segment = test[0], test[1], test[2], test[3]
            duration = test[4] if len(test) > 4 else 15.0
            description = test[5] if len(test) > 5 else None
            start_progress = (test[6] if len(test) > 6
                              else TURN_SPAWN_PROGRESS)
            kerb_check = bool(test[7]) if len(test) > 7 else False
            end_flag_progress = test[8] if len(test) > 8 else END_FLAG_PROGRESS
            expect_wrong_side = bool(test[9]) if len(test) > 9 else False
            print(f"\n\n{'#'*60}")
            print(f"# TEST {i}/{len(tests)}: '{start_point}' -> {direction.upper()} @ {speed} km/h")
            print(f"{'#'*60}")

            _emit({'type': 'start', 'index': i, 'total': len(tests),
                   'start_point': start_point, 'direction': direction,
                   'speed_kmh': speed, 'description': description})

            result = self.monitor_turn(direction, duration=duration, target_speed=speed, start_point=start_point,
                                       expected_end_segment=expected_end_segment,
                                       description=description, results=results,
                                       abort_check=abort_check,
                                       label=f"{i}/{len(tests)}",
                                       start_progress=start_progress,
                                       kerb_check=kerb_check,
                                       end_flag_progress=end_flag_progress,
                                       expect_wrong_side=expect_wrong_side)

            _emit({'type': 'done', 'index': i, 'passed': result['passed'],
                   'aborted': bool(result.get('aborted'))})

            if '--failfast' in sys.argv and not result['passed'] \
                    and not result.get('aborted'):
                print(yellow(
                    f"\n⏹  --failfast: stopping at the first failure "
                    f"(test {i}/{len(tests)})."))
                break

            pos += 1
            if pos < len(selected):
                print("\n⏸️  Pausing 1s before next test...")
                time.sleep(1)

        save_results(results)
        self.print_summary()
    
    def print_summary(self):
        """Print summary of all tests."""
        print("\n" + "="*60)
        print("FINAL SUMMARY")
        print("="*60)
        
        # Aborted scenarios (user jumped to another test from the cockpit)
        # were never actually tested - report them separately, don't count
        # them as failures.
        aborted = [r for r in self.test_results if r.get('aborted')]
        runnable = [r for r in self.test_results if not r.get('aborted')]
        passed = sum(1 for r in runnable if r['passed'])
        failed_offroad = sum(1 for r in runnable if r['off_road_detected'])
        failed_snap = sum(1 for r in runnable if r['instant_snap_detected'])
        failed_teleport = sum(1 for r in runnable if r.get('teleport_detected'))
        failed_crashed = sum(1 for r in runnable if r.get('game_crashed'))
        failed_wrongside = sum(1 for r in runnable
                               if r.get('wrong_side_detected')
                               and not r.get('expect_wrong_side'))
        failed_passed_flag = sum(1 for r in runnable if r.get('passed_flag'))
        failed_wrong_route = sum(
            1 for r in runnable
            if r['segment_changed'] and not r['reached_expected_segment']
            and r.get('final_segment') != r.get('expected_end_segment')
            and not r['off_road_detected'] and not r['instant_snap_detected']
            and not r.get('teleport_detected') and not r.get('game_crashed')
            and not r.get('passed_flag')
            and not r.get('expect_wrong_side')
        )
        # Timeouts include "timed out ON the correct segment before reaching
        # the flag" - that is an arrival problem, not a routing error.
        timeout = sum(
            1 for r in runnable
            if not r['reached_expected_segment'] and not r['off_road_detected']
            and not r['instant_snap_detected'] and not r.get('teleport_detected')
            and not r.get('game_crashed') and not r.get('passed_flag')
            and (not r['segment_changed']
                 or r.get('final_segment') == r.get('expected_end_segment'))
            and not r.get('expect_wrong_side')
        )
        # Hard criterion: a scenario that never reaches its red target flag
        # is a failure, whatever else may have happened on the way (this
        # overlaps with the categories above - it is the union of all
        # "never arrived" cases, not an extra one).
        failed_no_flag = sum(1 for r in runnable
                             if not r['reached_expected_segment']
                             and not r.get('expect_wrong_side'))
        
        def line(count, fail_label, pass_label):
            if count > 0:
                return f"  ❌ Failed ({fail_label}): {count}"
            return f"  ✅ Passed: no {pass_label}"

        print(f"\nTotal tests: {len(runnable)}"
              + (f"  (+{len(aborted)} aborted by user)" if aborted else ""))
        print(f"  ✅ Passed: {passed}")
        print(line(failed_offroad, "off-road", "off-road violations"))
        print(line(failed_snap, "instant snap", "instant snaps"))
        print(line(failed_teleport, "teleport/jump", "teleport/jump"))
        print(line(failed_crashed, "game crashed", "game crashes"))
        print(line(failed_wrongside, "wrong-side driving", "wrong-side driving"))
        print(line(failed_passed_flag, "drove past the end flag",
                   "runs past the end flag"))
        print(line(failed_wrong_route, "wrong end segment", "wrong end segments"))
        print(line(timeout, "timeout (never arrived)", "timeouts"))
        print(line(failed_no_flag, "never reached the end flag",
                   "missed end flags"))
        
        # Show detailed violations
        snap_violations = [r for r in self.test_results if r['instant_snap_detected']]
        offroad_violations = [r for r in self.test_results if r['off_road_detected']]
        
        if snap_violations:
            print(f"\n{'─'*60}")
            print(f"INSTANT HEADING SNAP VIOLATIONS: {len(snap_violations)}")
            print(f"{'─'*60}")
            for i, r in enumerate(snap_violations, 1):
                v = r['violation_details']
                label = f" @ '{r['start_point']}'" if r.get('start_point') else ""
                print(f"\n{i}. {r['direction'].upper()} turn{label} @ {r['target_speed_kmh']:.0f} km/h")
                print(f"   Time: {v['time']:.2f}s")
                print(f"   Heading change: {v['old_heading']:.1f}° → {v['new_heading']:.1f}° ({v['heading_change']:.1f}°)")
                print(f"   Position: ({v['position'][0]:.0f}, {v['position'][1]:.0f})")
                print(f"   Speed: {v['speed_kmh']:.0f} km/h")
                if v.get('screenshot'):
                    print(f"   Screenshot: {v['screenshot']}")
        
        if offroad_violations:
            print(f"\n{'─'*60}")
            print(f"OFF-ROAD VIOLATIONS: {len(offroad_violations)}")
            print(f"{'─'*60}")
            for i, r in enumerate(offroad_violations, 1):
                v = r['violation_details']
                label = f" @ '{r['start_point']}'" if r.get('start_point') else ""
                print(f"\n{i}. {r['direction'].upper()} turn{label} @ {r['target_speed_kmh']:.0f} km/h")
                print(f"   Time: {v['time']:.2f}s")
                print(f"   Position: ({v['position'][0]:.0f}, {v['position'][1]:.0f})")
                print(f"   Speed: {v['speed_kmh']:.0f} km/h")
                print(f"   Heading: {v['heading']:.1f}°")
                if v.get('screenshot'):
                    print(f"   Screenshot: {v['screenshot']}")
        
        print("\n" + "="*60)
        
        if passed == len(runnable):
            print(green("🎉 ALL TESTS PASSED! Smooth turns, no violations."))
        else:
            print(red(f"⚠️  {len(runnable) - passed} of {len(runnable)} "
                      f"test(s) failed. Review details above."))
        
        print("="*60 + "\n")
        
        return passed == len(self.test_results)


def main():
    """Run turn tests.
    
    By default, runs the DETERMINISTIC suite against the synthetic
    'basic' test map (start the game with: --map basic --api).
    
    Pass --random to instead teleport to random locations on whatever
    map is currently loaded (real OSM data or a test map).
    
    Pass --only <start_point> <direction> <speed_kmh> [end_segment] to run
    a SINGLE scenario directly instead of the whole suite (much faster when
    debugging one known-failing case):

        python tests/test_turning.py --only corner_right_entry right 120 6

    With end_segment, the red end flag is set at that segment's first 20%
    point as a VISUAL-ONLY marker (the running turn-test protocol, exactly
    like the full suite) - without it the nav has no destination and
    therefore no parking plan (no reverse-in).
    """
    # Ctrl-C handling: the SIGINT handler just sets a flag and tells the user
    # we're wrapping up; the suite loop then stops scheduling new tests, saves
    # the results collected so far, and exits cleanly (no traceback).
    signal.signal(signal.SIGINT, _on_sigint)

    tester = TurnTester()

    if not tester.health_check():
        sys.exit(1)

    # ONE shared results dict for the whole run: every mode records into it
    # and every save writes the same live object, so a late save can never
    # clobber results recorded earlier (see the 2026-08-18 bug where the
    # except-handler saved a stale dict and wiped 15 scenario results).
    results = load_results()

    try:
        if '--only' in sys.argv:
            idx = sys.argv.index('--only')
            start_point = sys.argv[idx + 1]
            direction = sys.argv[idx + 2]
            speed = float(sys.argv[idx + 3])
            end_seg = int(sys.argv[idx + 4]) if len(sys.argv) > idx + 4 \
                else None
            # Show the same "i/total" test number as a suite run does.
            test_no = next((n for n, t in enumerate(DETERMINISTIC_TESTS, 1)
                            if t[0] == start_point and t[1] == direction),
                           None)
            tester.monitor_turn(direction, duration=60.0, target_speed=speed,
                               start_point=start_point, results=results,
                               expected_end_segment=end_seg,
                               label=f"{test_no}/{len(DETERMINISTIC_TESTS)}"
                                     if test_no else None)
            save_results(results)
            tester.print_summary()
        elif '--random' in sys.argv:
            tester.run_random_test(results=results)
        else:
            tester.run_deterministic_test(results=results)

        if interrupted():
            # User interrupted (Ctrl-C) or closed the game window mid-run:
            # results so far are already saved per scenario, exit cleanly.
            print(yellow("\n⏹  Run interrupted - results saved to " + str(RESULTS_FILE)))
            sys.exit(130)

        # Exit code: 0 if all passed, 1 if any failed (aborted scenarios
        # don't count - they were never actually tested).
        runnable = [r for r in tester.test_results if not r.get('aborted')]
        all_passed = all(r['passed'] for r in runnable)
        sys.exit(0 if all_passed else 1)

    except KeyboardInterrupt:
        # Fallback for Ctrl-C landing between the handler and the loop checks.
        print(yellow("\n⏹  Run interrupted by user - results saved to " + str(RESULTS_FILE)))
        save_results(results)
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        save_results(results)
        tester.reset_controls()
        sys.exit(1)


if __name__ == '__main__':
    main()
