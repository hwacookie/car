#!/usr/bin/env python3
"""
Comprehensive Turn Testing via REST API
Tests both left and right turns with detailed monitoring
"""

import os
import requests
import time
import sys
import json
import signal
from datetime import datetime
from pathlib import Path


API_URL = os.environ.get("CAR_API_URL", "http://127.0.0.1:5000")  # explicit IPv4: 'localhost' may resolve to ::1, where macOS ControlCenter squats on :5000

# Results are persisted next to this script, keyed by "start_point|direction",
# so the next run can report whether each scenario passed the last time it ran
# (and why it failed, if it didn't). See docs/SPEC.md ("Turn Test Output").
RESULTS_FILE = Path(__file__).resolve().parent / "turning_results.json"
HISTORY_LIMIT = 500  # cap the per-scenario history so the file can't grow unbounded
# Signal the turn (blinker) only this many metres before the junction -
# like a real driver, not the instant the test starts.
SIGNAL_DISTANCE_M = 50.0


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

def select_tests(tests: list) -> list:
    """Pick which scenarios to run, keeping their ORIGINAL numbering.

        --tests 3              one scenario
        --tests 3,7,12         several
        --tests 3-6            an inclusive range
        --tests tjunction_from_top      every scenario at that start point
        --failed               whatever failed on the last recorded run

    Returns [(original_index, test), ...] so the printed "TEST 7/18" still
    identifies the same scenario however the suite is filtered - the point
    being to jump straight back to one that failed without renumbering it.
    """
    numbered = list(enumerate(tests, 1))

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

    wanted_idx: set[int] = set()
    wanted_name: set[str] = set()
    for tok in sys.argv[idx + 1].split(','):
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
    if result.get("game_crashed"):
        return "game process ended mid-test (window closed or physics watchdog crash)"
    if result.get("teleport_detected"):
        return "teleported / unexpected jump detected"
    if result.get("instant_snap_detected"):
        return "instant heading snap (unrealistic rotation)"
    if result.get("off_road_detected") or result.get('validator_violations', 0) > 0:
        return "cut the corner and drove off the road"
    if result.get("segment_changed") and not result.get("reached_expected_segment"):
        return (f"took the wrong route (ended on segment {result.get('final_segment')}, "
                f"expected {result.get('expected_end_segment')})")
    if result.get("reached_expected_segment") and not result.get("stopped_at_end"):
        return "reached the destination but never came to a clean stop"
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
}


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
    
    def send_control(self, **kwargs):
        """Send control inputs."""
        requests.post(f"{API_URL}/control", json=kwargs)
    
    def create_car_at_random(self):
        """Replace car with a fresh one at random location."""
        requests.post(f"{API_URL}/teleport", json={'random': True})
        time.sleep(0.3)

    def create_car_at_start_point(self, name: str):
        """Replace car with a fresh one at named start point."""
        requests.post(f"{API_URL}/teleport", json={'start_point': name})
        time.sleep(0.3)
    
    def set_hud_label(self, text: str | None):
        """Show (or clear) a short text label in the game's HUD."""
        requests.post(f"{API_URL}/label", json={'text': text})
    
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
                                        max_extra_time: float = 25.0):
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
    
    def monitor_turn(self, direction: str, duration: float = 15.0, target_speed: float = 50.0,
                      start_point: str | None = None, expected_end_segment: int | None = None,
                      description: str | None = None, results: dict | None = None) -> dict:
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
                roundabout: entry -> ring -> ring -> exit) until this
                exact segment is reached, a violation occurs, or duration
                runs out. If None (only used for the legacy random-
                location suite, where there's no way to know the correct
                answer in advance), falls back to the old "any change"
                behavior.
        
        Returns:
            dict with test results
        """
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
        
        # Create a fresh car at the given start point or a random location
        if start_point:
            print(f"📍 Creating new car at '{start_point}'...")
            self.create_car_at_start_point(start_point)
            number = START_POINT_NUMBER.get(start_point)
            self.set_hud_label(str(number) if number is not None else start_point)
        else:
            print("📍 Creating new car at random location...")
            self.create_car_at_random()
            self.set_hud_label(None)
        
        # If the game dies during setup (window closed, or the teleport
        # watchdog crashing the process right after the teleport), treat it
        # exactly like a crash mid-monitor: record game_crashed for this
        # scenario and skip straight to the summary instead of raising.
        setup_crashed = False
        try:
            state = self.get_state()
            initial_segment = state['segment']
            initial_pos = (state['x'], state['y'])
            initial_heading = state['heading']
            print(f"   Starting at segment {initial_segment}")
            print(f"   Position: ({state['x']:.0f}, {state['y']:.0f})")
            print(f"   Heading: {state['heading']:.1f}°")
            
            # Accelerate to target speed (no blinker yet - we signal only
            # once we are actually close to the junction, like a real driver)
            print(f"🚗 Accelerating to {target_speed:.0f} km/h...")
            self.send_control(accelerate=True)
            
            # Wait to reach speed
            for _ in range(50):  # 5 seconds max
                state = self.get_state()
                if state['speed_kmh'] >= target_speed * 0.9:
                    break
                time.sleep(0.1)
            
            print(f"   Reached {state['speed_kmh']:.0f} km/h")
            
            if direction != 'straight':
                # Wait until we are SIGNAL_DISTANCE_M or less before the
                # junction, then signal the turn.
                blinker_key = 'blinker_left' if direction == 'left' else 'blinker_right'
                dist = state.get('distance_to_junction')
                for _ in range(100):  # 10 seconds max
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
        off_road_detected = False
        instant_snap_detected = False
        teleport_detected = False
        game_crashed = False
        stopped_ok = False
        violation_details = None
        positions = []
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
        
        while not setup_crashed and time.time() - start_time < duration:
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
            
            # Check for off-road violation (live check + validator log)
            if not state['on_road'] or state.get('validator_violations', 0) > 0:
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
            
            # Check if segment changed. If we have a SPECIFIC expected
            # end segment, only that counts as arrival - a mere change to
            # some OTHER segment (e.g. a wrong turn) is noted but keeps
            # monitoring (the car might still be mid-maneuver, as on a
            # roundabout: entry segment -> ring -> ring -> exit segment -
            # only the last of those is "done"). Without an expected
            # segment (legacy random-location mode only), any change at
            # all is accepted, same as before.
            if state['segment'] != initial_segment:
                if not segment_changed:
                    segment_changed = True
                    print(f"\n   ℹ️  Segment changed: {initial_segment} → {state['segment']} "
                          f"(t={time.time() - start_time:.2f}s)")
                if expected_end_segment is None or state['segment'] == expected_end_segment:
                    reached_expected_segment = True
                    print(f"\n   ✅ Reached designated end segment {state['segment']}!")
                    print(f"      Time: {time.time() - start_time:.2f}s")
                    print(f"      Max heading change per frame: {max_heading_change_per_frame:.1f}°")
                    # Don't just stop watching the instant we arrive on
                    # the target segment - actually drive all the way to
                    # ITS far end and come to a stop there, so "arrived"
                    # means a real, completed, parked maneuver (matching
                    # what a human driver would do: pull all the way in
                    # and stop, not abandon the car halfway down the new
                    # road).
                    final_pos, stopped_ok, stop_details = self._drive_to_segment_end_and_stop(
                        state['segment'], start_time
                    )
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
        
        # Stop car (best-effort - the game may have crashed, in which
        # case this is just a no-op rather than another unhandled error)
        try:
            self.reset_controls()
        except requests.exceptions.RequestException:
            pass
        
        passed = (
            reached_expected_segment
            and stopped_ok
            and not off_road_detected
            and not instant_snap_detected
            and not teleport_detected
            and not game_crashed
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
            'off_road_detected': off_road_detected,
            'instant_snap_detected': instant_snap_detected,
            'teleport_detected': teleport_detected,
            'game_crashed': game_crashed,
            'max_heading_change_per_frame': max_heading_change_per_frame,
            'violation_details': violation_details,
            'positions': positions,
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

        # Validator violations (off-road caught between API polls)
        vv = state.get('validator_violations', 0)
        if vv > 0:
            print(red(f"   ⚠️  Validator: {vv} off-road violation(s) logged"))

        # Lane guard stats
        lg = state.get('lane_guard_stats', {})
        wrong_frames = lg.get('wrong_side_frames', 0)
        wrong_secs = lg.get('wrong_side_seconds', 0.0)
        if wrong_frames > 0:
            print(red(f"   ⚠️  Wrong-side driving: {wrong_frames} frames ({wrong_secs}s)"))
        else:
            print(green("   ✅ Lane guard: no wrong-side driving detected"))

        # Colored one-line verdict: green "passed" or red "fail: <reason>".
        reason = describe_failure(result)
        if reason is None:
            print(green("   ✅ PASSED"))
            print(dim("      Reached designated end segment, drove to its end and "
                      "stopped there, stayed on road, no violations"))
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
    
    def run_deterministic_test(self, results: dict | None = None):
        """Run the turn test suite against KNOWN, reproducible scenarios
        from the 'basic' synthetic test map (see src/test_maps.py).
        Requires the game to be started with: --map basic --api
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

        # (start_point, direction, speed_kmh, expected_end_segment, duration=15.0)
        #
        # speed_kmh is just how fast we wait to reach before we start
        # watching - NOT a fixed cruising/cornering speed. In RAILS mode
        # the car always accelerates toward top speed whenever the
        # accelerator is held, except while actively executing a turn's
        # arc (capped to that arc's own planned speed); the automatic
        # pre-turn braking logic is what actually slows it down in time
        # for the corner, then it goes right back to accelerating flat
        # out afterwards. So testing the same corner at 30/50/80 km/h
        # doesn't exercise different driving behavior - it's the same
        # "floor it, brake only as needed for the corner" behavior every
        # time - hence one run per corner is enough.
        #
        # expected_end_segment is REQUIRED (not just "did some segment
        # change happen") - a test only really passes if the car actually
        # arrives at its designated destination segment. Without this,
        # e.g. a LEFT turn that actually went RIGHT would still "pass" as
        # long as it ended up SOMEWHERE else - which is exactly the kind
        # of bug (API-driven blinkers not reaching the driver's actual
        # routing logic) that slipped through before this was added.
        # These segment indices come from src/test_maps.py's
        # build_basic_test_map() and were verified against actual runs.
        # NOTE on left/right: the test map now uses the OSM coordinate
        # system (Y grows north, same as the real OSM map), so the
        # handedness of junctions is the reverse of the original map
        # (which had Y growing south). The expected end segments below were
        # re-verified against actual bicycle-mode runs on the new map.
        # Tuple shape: (start_point, direction, speed_kmh, expected_end_segment,
        #               [duration_s], [one-line human description])
        tests = [
            # 90-degree corners (the classic reported bug)
            ('corner_right_entry', 'right', 80, 2, 15.0,
             "Taking a sharp 90° right corner"),
            ('corner_left_entry', 'left', 80, 4, 15.0,
             "Taking a sharp 90° left corner"),
            # T-junction (perpendicular 3-way)
            ('tjunction_from_top', 'left', 80, 7, 15.0,
             "T-junction: turning left off the stem"),
            ('tjunction_from_top', 'right', 80, 6, 15.0,
             "T-junction: turning right off the stem"),
            # Y-intersection (shallow ~40 degree diverging angles)
            ('y_from_stem', 'left', 80, 10, 15.0,
             "Y-intersection: taking the left fork"),
            ('y_from_stem', 'right', 80, 9, 15.0,
             "Y-intersection: taking the right fork"),
            # 4-way crossroads
            ('crossroads_from_north', 'left', 80, 14, 15.0,
             "4-way crossroads: turning left"),
            ('crossroads_from_north', 'right', 80, 13, 15.0,
             "4-way crossroads: turning right"),
            ('crossroads_from_north', 'straight', 80, 12, 15.0,
             "4-way crossroads: going straight through"),
            # One-way street (legal direction)
            ('oneway_entry', 'straight', 80, 16, 15.0,
             "Entering a one-way street in the legal direction"),
            # Simple curves (degree-2 nodes, no blinker needed). The S-curve
            # is ~470 m long, so at cruise (~58 km/h) the car needs ~30 s to
            # traverse it - longer than the default 15 s monitor window, hence
            # the duration override.
            ('s_curve', 'straight', 80, 20, 40.0,
             "Following a zig-zag S-curve road"),
            ('hairpin_entry', 'straight', 80, 25, 15.0,
             "Driving a hairpin bend (entry side)"),
            ('sweeping_curve', 'straight', 80, 27, 15.0,
             "Following a wide, sweeping curve"),
            # Hairpin, entered from the opposite end (reverse direction)
            ('hairpin_exit', 'straight', 80, 24, 15.0,
             "Driving a hairpin bend (exit side)"),
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
            # (seg 28). Monitoring keeps running THROUGH the intermediate ring
            # segments instead of stopping at the first one, since only 28
            # (the actual exit) counts as arrival. Takes longer than a normal
            # turn (~25s to go most of the way around before exiting), hence
            # the longer duration override.
            ('roundabout_from_north', 'right', 40, 28, 30.0,
             "Circling the roundabout and taking the first exit (west)"),
            # Sliver junction (the real-world segment-815 layout: a 4.16 m
            # approach stub meeting a 3-way junction where one exit is a
            # near-90-degree turn). The car must get through the tiny
            # stub and onto the correct exit without clipping the junction.
            # NOTE: segment indices shifted from 41/42/43 to 97/96/99 due to
            # the 64-node roundabout ring adding many segments before the
            # sliver junction.
            ('sliver_approach', 'straight', 80, 97, 15.0,
             "Sliver junction: going straight through the tiny approach stub"),
            ('sliver_approach', 'right', 80, 98, 15.0,
             "Sliver junction: turning right off the tiny approach stub"),
            ('sliver_approach', 'left', 80, 99, 15.0,
             "Sliver junction: turning left off the tiny approach stub"),
        ]

        if results is None:
            results = load_results()
        print(f"\n📄 Results file: {RESULTS_FILE}")
        print(f"   (each scenario's outcome is saved there; next run shows its last result)")

        selected = select_tests(tests)
        if not selected:
            return
        if len(selected) != len(tests):
            print(f"\n▶  Running {len(selected)} of {len(tests)} scenarios: "
                  f"{', '.join(str(i) for i, _ in selected)}")

        for i, test in selected:
            # User interrupted (Ctrl-C) or closed the game window: stop
            # scheduling more tests now - results so far are already saved.
            if interrupted():
                print(yellow(f"\n⏸️  Stopping after test {i-1}/{len(tests)} - "
                             f"run interrupted (Ctrl-C or game window closed)."))
                break

            start_point, direction, speed, expected_end_segment = test[0], test[1], test[2], test[3]
            duration = test[4] if len(test) > 4 else 15.0
            description = test[5] if len(test) > 5 else None
            print(f"\n\n{'#'*60}")
            print(f"# TEST {i}/{len(tests)}: '{start_point}' -> {direction.upper()} @ {speed} km/h")
            print(f"{'#'*60}")

            self.monitor_turn(direction, duration=duration, target_speed=speed, start_point=start_point,
                               expected_end_segment=expected_end_segment,
                               description=description, results=results)

            if '--failfast' in sys.argv and self.test_results and \
                    not self.test_results[-1]['passed']:
                print(yellow(
                    f"\n⏹  --failfast: stopping at the first failure "
                    f"(test {i}/{len(tests)})."))
                break

            if (i, test) != selected[-1]:
                print("\n⏸️  Pausing 1s before next test...")
                time.sleep(1)

        save_results(results)
        self.print_summary()
    
    def print_summary(self):
        """Print summary of all tests."""
        print("\n" + "="*60)
        print("FINAL SUMMARY")
        print("="*60)
        
        passed = sum(1 for r in self.test_results if r['passed'])
        failed_offroad = sum(1 for r in self.test_results if r['off_road_detected'])
        failed_snap = sum(1 for r in self.test_results if r['instant_snap_detected'])
        failed_teleport = sum(1 for r in self.test_results if r.get('teleport_detected'))
        failed_crashed = sum(1 for r in self.test_results if r.get('game_crashed'))
        failed_wrong_route = sum(
            1 for r in self.test_results
            if r['segment_changed'] and not r['reached_expected_segment']
            and not r['off_road_detected'] and not r['instant_snap_detected']
            and not r.get('teleport_detected') and not r.get('game_crashed')
        )
        timeout = sum(
            1 for r in self.test_results
            if not r['reached_expected_segment'] and not r['off_road_detected']
            and not r['instant_snap_detected'] and not r.get('teleport_detected')
            and not r.get('game_crashed') and not r['segment_changed']
        )
        
        def line(count, fail_label, pass_label):
            if count > 0:
                return f"  ❌ Failed ({fail_label}): {count}"
            return f"  ✅ Passed: no {pass_label}"

        print(f"\nTotal tests: {len(self.test_results)}")
        print(f"  ✅ Passed: {passed}")
        print(line(failed_offroad, "off-road", "off-road violations"))
        print(line(failed_snap, "instant snap", "instant snaps"))
        print(line(failed_teleport, "teleport/jump", "teleport/jump"))
        print(line(failed_crashed, "game crashed", "game crashes"))
        print(line(failed_wrong_route, "wrong end segment", "wrong end segments"))
        print(line(timeout, "timeout (never arrived)", "timeouts"))
        
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
        
        if passed == len(self.test_results):
            print(green("🎉 ALL TESTS PASSED! Smooth turns, no violations."))
        else:
            print(red(f"⚠️  {len(self.test_results) - passed} of {len(self.test_results)} "
                      f"test(s) failed. Review details above."))
        
        print("="*60 + "\n")
        
        return passed == len(self.test_results)


def main():
    """Run turn tests.
    
    By default, runs the DETERMINISTIC suite against the synthetic
    'basic' test map (start the game with: --map basic --api).
    
    Pass --random to instead teleport to random locations on whatever
    map is currently loaded (real OSM data or a test map).
    
    Pass --only <start_point> <direction> <speed_kmh> to run a SINGLE
    scenario directly instead of the whole suite (much faster when
    debugging one known-failing case):
    
        python tests/test_turning.py --only corner_right_entry right 120
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
            tester.monitor_turn(direction, duration=15.0, target_speed=speed,
                               start_point=start_point, results=results)
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

        # Exit code: 0 if all passed, 1 if any failed
        all_passed = all(r['passed'] for r in tester.test_results)
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
