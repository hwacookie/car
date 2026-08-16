#!/usr/bin/env python3
"""
Comprehensive Turn Testing via REST API
Tests both left and right turns with detailed monitoring
"""

import requests
import time
import sys
from pathlib import Path


API_URL = "http://localhost:5000"

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
        """Reset all control inputs."""
        requests.post(f"{API_URL}/reset")
    
    def get_state(self):
        """Get current game state."""
        response = requests.get(f"{API_URL}/state")
        return response.json()
    
    def send_control(self, **kwargs):
        """Send control inputs."""
        requests.post(f"{API_URL}/control", json=kwargs)
    
    def teleport_random(self):
        """Teleport to random location."""
        requests.post(f"{API_URL}/teleport", json={'random': True})
        time.sleep(0.3)  # Wait for teleport to complete
    
    def teleport_to_start_point(self, name: str):
        """Teleport to a deterministic named start point (synthetic test maps)."""
        requests.post(f"{API_URL}/teleport", json={'start_point': name})
        time.sleep(0.3)  # Wait for teleport to complete
    
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
                details['violation_details'] = {'type': 'off_road', 'position': pos}
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
                      start_point: str | None = None, expected_end_segment: int | None = None) -> dict:
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
        print(f"\n{'='*60}")
        print(f"Testing {direction.upper()} Turn" + (f" @ '{start_point}'" if start_point else ""))
        print(f"{'='*60}")
        
        # Reset and enable breadcrumbs for visual debugging
        self.reset_controls()
        requests.post(f"{API_URL}/toggle", json={'breadcrumbs': True})
        
        # Teleport to a deterministic start point if given, else random location
        if start_point:
            print(f"📍 Teleporting to named start point '{start_point}'...")
            self.teleport_to_start_point(start_point)
            number = START_POINT_NUMBER.get(start_point)
            self.set_hud_label(str(number) if number is not None else start_point)
        else:
            print("📍 Teleporting to random location...")
            self.teleport_random()
            self.set_hud_label(None)
        
        state = self.get_state()
        initial_segment = state['segment']
        initial_pos = (state['x'], state['y'])
        initial_heading = state['heading']
        print(f"   Starting at segment {initial_segment}")
        print(f"   Position: ({state['x']:.0f}, {state['y']:.0f})")
        print(f"   Heading: {state['heading']:.1f}°")
        
        # Accelerate to target speed
        print(f"🚗 Accelerating to {target_speed:.0f} km/h...")
        if direction == 'straight':
            self.send_control(accelerate=True)
        else:
            blinker_key = 'blinker_left' if direction == 'left' else 'blinker_right'
            self.send_control(accelerate=True, **{blinker_key: True})
        
        # Wait to reach speed
        for _ in range(50):  # 5 seconds max
            state = self.get_state()
            if state['speed_kmh'] >= target_speed * 0.9:
                break
            time.sleep(0.1)
        
        print(f"   Reached {state['speed_kmh']:.0f} km/h")
        print(f"   {direction.upper()} blinker activated")
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
        
        while time.time() - start_time < duration:
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
            
            # Check for off-road violation
            if not state['on_road']:
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
        
        if result['passed']:
            print(f"   ✅ TEST PASSED: Reached designated end segment, drove to its end "
                  f"and stopped there, stayed on road, no violations")
        elif game_crashed:
            print(f"   ❌ TEST FAILED: Game process crashed / connection lost mid-test")
        elif teleport_detected:
            print(f"   ❌ TEST FAILED: Teleportation/unexpected jump detected")
        elif instant_snap_detected:
            print(f"   ❌ TEST FAILED: Instant heading snap detected")
        elif off_road_detected:
            print(f"   ❌ TEST FAILED: Car went off-road")
        elif expected_end_segment is not None and segment_changed and not reached_expected_segment:
            print(f"   ❌ TEST FAILED: Ended on segment {state['segment']}, "
                  f"expected {expected_end_segment} (wrong turn/route!)")
        elif reached_expected_segment and not stopped_ok:
            print(f"   ❌ TEST FAILED: Reached the end segment but never came to a "
                  f"clean stop at its far end")
        else:
            print(f"   ⚠️  TEST TIMEOUT: Never reached the designated end segment in {duration}s")
        
        print(f"{'─'*60}\n")
        
        self.test_results.append(result)
        
        return result
    
    def run_random_test(self):
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
    
    def run_deterministic_test(self):
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
        tests = [
            # 90-degree corners (the classic reported bug)
            ('corner_right_entry', 'right', 80, 2),
            ('corner_left_entry', 'left', 80, 4),
            # T-junction (perpendicular 3-way)
            ('tjunction_from_top', 'left', 80, 7),
            ('tjunction_from_top', 'right', 80, 6),
            # Y-intersection (shallow ~40 degree diverging angles)
            ('y_from_stem', 'left', 80, 10),
            ('y_from_stem', 'right', 80, 9),
            # 4-way crossroads
            ('crossroads_from_north', 'left', 80, 14),
            ('crossroads_from_north', 'right', 80, 13),
            ('crossroads_from_north', 'straight', 80, 12),
            # One-way street (legal direction)
            ('oneway_entry', 'straight', 80, 16),
            # Simple curves (degree-2 nodes, no blinker needed). The S-curve
            # is ~470 m long, so at cruise (~58 km/h) the car needs ~30 s to
            # traverse it - longer than the default 15 s monitor window, hence
            # the duration override.
            ('s_curve', 'straight', 80, 20, 40.0),
            ('hairpin_entry', 'straight', 80, 25),
            ('sweeping_curve', 'straight', 80, 27),
            # Hairpin, entered from the opposite end (reverse direction)
            ('hairpin_exit', 'straight', 80, 24),
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
            ('roundabout_from_north', 'right', 40, 28, 30.0),
            # Sliver junction (the real-world segment-815 layout: a 4.16 m
            # approach stub meeting a 3-way junction where one exit is a
            # near-90-degree turn). The car must get through the tiny
            # stub and onto the correct exit without clipping the junction.
            # NOTE: segment indices shifted from 41/42/43 to 97/96/99 due to
            # the 64-node roundabout ring adding many segments before the
            # sliver junction.
            ('sliver_approach', 'straight', 80, 97),
            ('sliver_approach', 'right', 80, 98),
            ('sliver_approach', 'left', 80, 99),
        ]

        for i, test in enumerate(tests, 1):
            start_point, direction, speed, expected_end_segment = test[0], test[1], test[2], test[3]
            duration = test[4] if len(test) > 4 else 15.0
            print(f"\n\n{'#'*60}")
            print(f"# TEST {i}/{len(tests)}: '{start_point}' -> {direction.upper()} @ {speed} km/h")
            print(f"{'#'*60}")

            self.monitor_turn(direction, duration=duration, target_speed=speed, start_point=start_point,
                               expected_end_segment=expected_end_segment)

            if i < len(tests):
                print("\n⏸️  Pausing 1s before next test...")
                time.sleep(1)

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
        
        print(f"\nTotal tests: {len(self.test_results)}")
        print(f"  ✅ Passed: {passed}")
        print(f"  ❌ Failed (off-road): {failed_offroad}")
        print(f"  ❌ Failed (instant snap): {failed_snap}")
        print(f"  ❌ Failed (teleport/jump): {failed_teleport}")
        print(f"  ❌ Failed (game crashed): {failed_crashed}")
        print(f"  ❌ Failed (wrong end segment): {failed_wrong_route}")
        print(f"  ⚠️  Timeout (never arrived): {timeout}")
        
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
        
        if failed_offroad == 0 and failed_snap == 0:
            print("🎉 ALL TESTS PASSED! Smooth turns, no violations.")
        else:
            print(f"⚠️  {failed_offroad + failed_snap} test(s) failed. Review details above.")
        
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
    tester = TurnTester()
    
    if not tester.health_check():
        sys.exit(1)
    
    try:
        if '--only' in sys.argv:
            idx = sys.argv.index('--only')
            start_point = sys.argv[idx + 1]
            direction = sys.argv[idx + 2]
            speed = float(sys.argv[idx + 3])
            tester.monitor_turn(direction, duration=15.0, target_speed=speed, start_point=start_point)
            tester.print_summary()
        elif '--random' in sys.argv:
            tester.run_random_test()
        else:
            tester.run_deterministic_test()
        
        # Exit code: 0 if all passed, 1 if any failed
        all_passed = all(r['passed'] for r in tester.test_results)
        sys.exit(0 if all_passed else 1)
        
    except KeyboardInterrupt:
        print("\n\n❌ Tests interrupted by user")
        tester.reset_controls()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        tester.reset_controls()
        sys.exit(1)


if __name__ == '__main__':
    main()
