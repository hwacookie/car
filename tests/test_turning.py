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
        self.violations = []
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
    
    def monitor_turn(self, direction: str, duration: float = 15.0, target_speed: float = 50.0) -> dict:
        """Monitor a turn for violations.
        
        Args:
            direction: "left" or "right"
            duration: Maximum time to monitor (seconds)
            target_speed: Target speed in km/h
        
        Returns:
            dict with test results
        """
        print(f"\n{'='*60}")
        print(f"Testing {direction.upper()} Turn")
        print(f"{'='*60}")
        
        # Reset and enable breadcrumbs for visual debugging
        self.reset_controls()
        requests.post(f"{API_URL}/toggle", json={'breadcrumbs': True})
        
        # Teleport to random location
        print("📍 Teleporting to random location...")
        self.teleport_random()
        
        state = self.get_state()
        initial_segment = state['segment']
        initial_pos = (state['x'], state['y'])
        print(f"   Starting at segment {initial_segment}")
        print(f"   Position: ({state['x']:.0f}, {state['y']:.0f})")
        print(f"   Heading: {state['heading']:.1f}°")
        
        # Accelerate to target speed
        print(f"🚗 Accelerating to {target_speed:.0f} km/h...")
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
        print(f"   Checking for off-road violations every frame...")
        
        # Monitor turn
        start_time = time.time()
        frames_checked = 0
        segment_changed = False
        off_road_detected = False
        violation_details = None
        positions = []
        
        while time.time() - start_time < duration:
            state = self.get_state()
            frames_checked += 1
            
            # Record position
            positions.append({
                'time': time.time() - start_time,
                'x': state['x'],
                'y': state['y'],
                'heading': state['heading'],
                'speed_kmh': state['speed_kmh'],
                'segment': state['segment'],
                'on_road': state['on_road']
            })
            
            # Check for off-road violation
            if not state['on_road']:
                off_road_detected = True
                violation_details = {
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
                screenshot = self.save_violation_screenshot(direction, state)
                violation_details['screenshot'] = screenshot
                
                break
            
            # Check if segment changed (turn completed)
            if state['segment'] != initial_segment:
                segment_changed = True
                print(f"\n   ✅ Turn completed!")
                print(f"      Segment changed: {initial_segment} → {state['segment']}")
                print(f"      Time: {time.time() - start_time:.2f}s")
                print(f"      Distance traveled: {((state['x'] - initial_pos[0])**2 + (state['y'] - initial_pos[1])**2)**0.5:.0f} pixels")
                break
            
            time.sleep(0.05)  # Check at ~20 FPS
        
        # Stop car
        self.reset_controls()
        
        # Prepare results
        result = {
            'direction': direction,
            'target_speed_kmh': target_speed,
            'frames_checked': frames_checked,
            'duration': time.time() - start_time,
            'initial_segment': initial_segment,
            'final_segment': state['segment'],
            'segment_changed': segment_changed,
            'off_road_detected': off_road_detected,
            'violation_details': violation_details,
            'positions': positions,
            'passed': segment_changed and not off_road_detected
        }
        
        # Summary
        print(f"\n{'─'*60}")
        print(f"   Frames checked: {frames_checked}")
        print(f"   Duration: {result['duration']:.2f}s")
        
        if result['passed']:
            print(f"   ✅ TEST PASSED: Turn completed, stayed on road")
        elif off_road_detected:
            print(f"   ❌ TEST FAILED: Car went off-road")
        else:
            print(f"   ⚠️  TEST TIMEOUT: No segment change in {duration}s")
        
        print(f"{'─'*60}\n")
        
        self.test_results.append(result)
        if off_road_detected:
            self.violations.append(violation_details)
        
        return result
    
    def run_comprehensive_test(self):
        """Run full test suite with multiple speeds and directions."""
        print("\n" + "="*60)
        print("COMPREHENSIVE TURN TESTING")
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
    
    def print_summary(self):
        """Print summary of all tests."""
        print("\n" + "="*60)
        print("FINAL SUMMARY")
        print("="*60)
        
        passed = sum(1 for r in self.test_results if r['passed'])
        failed = sum(1 for r in self.test_results if r['off_road_detected'])
        timeout = sum(1 for r in self.test_results if not r['segment_changed'] and not r['off_road_detected'])
        
        print(f"\nTotal tests: {len(self.test_results)}")
        print(f"  ✅ Passed: {passed}")
        print(f"  ❌ Failed (off-road): {failed}")
        print(f"  ⚠️  Timeout (no turn): {timeout}")
        
        if self.violations:
            print(f"\n{'─'*60}")
            print(f"VIOLATIONS DETECTED: {len(self.violations)}")
            print(f"{'─'*60}")
            for i, v in enumerate(self.violations, 1):
                print(f"\n{i}. Time: {v['time']:.2f}s")
                print(f"   Position: ({v['position'][0]:.0f}, {v['position'][1]:.0f})")
                print(f"   Speed: {v['speed_kmh']:.0f} km/h")
                print(f"   Heading: {v['heading']:.1f}°")
                if v.get('screenshot'):
                    print(f"   Screenshot: {v['screenshot']}")
        
        print("\n" + "="*60)
        
        if failed == 0:
            print("🎉 ALL TESTS PASSED! No off-road violations detected.")
        else:
            print(f"⚠️  {failed} test(s) failed. Review screenshots above.")
        
        print("="*60 + "\n")
        
        return passed == len(self.test_results)


def main():
    """Run turn tests."""
    tester = TurnTester()
    
    if not tester.health_check():
        sys.exit(1)
    
    try:
        tester.run_comprehensive_test()
        
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
