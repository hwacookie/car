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
    
    def monitor_turn(self, direction: str, duration: float = 15.0, target_speed: float = 50.0,
                      start_point: str | None = None) -> dict:
        """Monitor a turn for violations.
        
        Args:
            direction: "left" or "right"
            duration: Maximum time to monitor (seconds)
            target_speed: Target speed in km/h
            start_point: If given, teleport to this deterministic named start
                point (synthetic test map) instead of a random location.
        
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
        else:
            print("📍 Teleporting to random location...")
            self.teleport_random()
        
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
        print(f"\n🔍 Monitoring turn for {duration}s...")
        print(f"   Checking for:")
        print(f"     - Off-road violations")
        print(f"     - Instant heading snaps (>30° per frame)")
        print(f"     - Smooth circular arc progression")
        
        # Monitor turn
        start_time = time.time()
        frames_checked = 0
        segment_changed = False
        off_road_detected = False
        instant_snap_detected = False
        violation_details = None
        positions = []
        last_heading = initial_heading
        max_heading_change_per_frame = 0.0
        
        while time.time() - start_time < duration:
            state = self.get_state()
            frames_checked += 1
            current_heading = state['heading']
            
            # Calculate heading change (handle 360° wrap)
            heading_diff = abs((current_heading - last_heading + 180) % 360 - 180)
            max_heading_change_per_frame = max(max_heading_change_per_frame, heading_diff)
            
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
            if heading_diff > 30.0:
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
            
            # Check if segment changed (turn completed)
            if state['segment'] != initial_segment:
                segment_changed = True
                print(f"\n   ✅ Turn completed!")
                print(f"      Segment changed: {initial_segment} → {state['segment']}")
                print(f"      Time: {time.time() - start_time:.2f}s")
                print(f"      Distance traveled: {((state['x'] - initial_pos[0])**2 + (state['y'] - initial_pos[1])**2)**0.5:.0f} pixels")
                print(f"      Max heading change per frame: {max_heading_change_per_frame:.1f}°")
                break
            
            last_heading = current_heading
            time.sleep(0.05)  # Check at ~20 FPS
        
        # Stop car
        self.reset_controls()
        
        # Prepare results
        result = {
            'start_point': start_point,
            'direction': direction,
            'target_speed_kmh': target_speed,
            'frames_checked': frames_checked,
            'duration': time.time() - start_time,
            'initial_segment': initial_segment,
            'final_segment': state['segment'],
            'segment_changed': segment_changed,
            'off_road_detected': off_road_detected,
            'instant_snap_detected': instant_snap_detected,
            'max_heading_change_per_frame': max_heading_change_per_frame,
            'violation_details': violation_details,
            'positions': positions,
            'passed': segment_changed and not off_road_detected and not instant_snap_detected
        }
        
        # Summary
        print(f"\n{'─'*60}")
        print(f"   Frames checked: {frames_checked}")
        print(f"   Duration: {result['duration']:.2f}s")
        print(f"   Max heading change per frame: {result['max_heading_change_per_frame']:.1f}°")
        
        if result['passed']:
            print(f"   ✅ TEST PASSED: Smooth turn completed, stayed on road")
        elif instant_snap_detected:
            print(f"   ❌ TEST FAILED: Instant heading snap detected")
        elif off_road_detected:
            print(f"   ❌ TEST FAILED: Car went off-road")
        else:
            print(f"   ⚠️  TEST TIMEOUT: No segment change in {duration}s")
        
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

        # (start_point, direction, speed_kmh)
        tests = [
            # 90-degree corners (the classic reported bug)
            ('corner_right_entry', 'right', 30),
            ('corner_right_entry', 'right', 50),
            ('corner_right_entry', 'right', 80),
            ('corner_left_entry', 'left', 30),
            ('corner_left_entry', 'left', 50),
            ('corner_left_entry', 'left', 80),
            # T-junction (perpendicular 3-way)
            ('tjunction_from_top', 'left', 50),
            ('tjunction_from_top', 'right', 50),
            # Y-intersection (shallow ~40 degree diverging angles)
            ('y_from_stem', 'left', 50),
            ('y_from_stem', 'right', 50),
            # 4-way crossroads
            ('crossroads_from_north', 'left', 50),
            ('crossroads_from_north', 'right', 50),
            ('crossroads_from_north', 'straight', 50),
            # One-way street (legal direction)
            ('oneway_entry', 'straight', 40),
            # Simple curves (degree-2 nodes, no blinker needed)
            ('s_curve', 'straight', 40),
            ('hairpin_entry', 'straight', 20),
            ('sweeping_curve', 'straight', 60),
        ]

        for i, (start_point, direction, speed) in enumerate(tests, 1):
            print(f"\n\n{'#'*60}")
            print(f"# TEST {i}/{len(tests)}: '{start_point}' -> {direction.upper()} @ {speed} km/h")
            print(f"{'#'*60}")

            self.monitor_turn(direction, duration=15.0, target_speed=speed, start_point=start_point)

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
        timeout = sum(1 for r in self.test_results if not r['segment_changed'] and not r['off_road_detected'] and not r['instant_snap_detected'])
        
        print(f"\nTotal tests: {len(self.test_results)}")
        print(f"  ✅ Passed: {passed}")
        print(f"  ❌ Failed (off-road): {failed_offroad}")
        print(f"  ❌ Failed (instant snap): {failed_snap}")
        print(f"  ⚠️  Timeout (no turn): {timeout}")
        
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
