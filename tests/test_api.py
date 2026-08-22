#!/usr/bin/env python3
"""
Test script for the REST API
Demonstrates remote control of the car game
"""

import requests
import time
import sys


API_URL = "http://127.0.0.1:5000"  # explicit IPv4: 'localhost' may resolve to ::1, where macOS ControlCenter squats on :5000


def test_health():
    """Test that API is running."""
    try:
        response = requests.get(f"{API_URL}/health", timeout=1)
        data = response.json()
        print("✅ Health check passed:", data)
        return True
    except requests.exceptions.RequestException as e:
        print(f"❌ Health check failed: {e}")
        print("   Make sure the game is running with --api flag")
        print("   Example: python -m src.main --api")
        return False


def get_state():
    """Get current game state."""
    response = requests.get(f"{API_URL}/state")
    return response.json()


def send_control(**kwargs):
    """Send control inputs."""
    response = requests.post(f"{API_URL}/control", json=kwargs)
    return response.json()


def reset_control():
    """Reset all controls to false."""
    response = requests.post(f"{API_URL}/reset")
    return response.json()


def teleport_random():
    """Teleport to random location."""
    response = requests.post(f"{API_URL}/teleport", json={'random': True})
    return response.json()


def toggle_breadcrumbs(enabled: bool):
    """Toggle breadcrumb trail."""
    response = requests.post(f"{API_URL}/toggle", json={'breadcrumbs': enabled})
    return response.json()


def test_basic_driving():
    """Test basic driving: accelerate, brake, turn."""
    print("\n🚗 Test: Basic Driving")
    print("=" * 50)
    
    # Get initial state
    state = get_state()
    print(f"Initial: Speed={state['speed_kmh']:.0f} km/h, Segment={state['segment']}")
    
    # Accelerate for 2 seconds
    print("Accelerating...")
    send_control(accelerate=True)
    time.sleep(2)
    
    state = get_state()
    print(f"After 2s: Speed={state['speed_kmh']:.0f} km/h")
    
    # Coast for 1 second
    print("Coasting...")
    reset_control()
    time.sleep(1)
    
    state = get_state()
    print(f"After coast: Speed={state['speed_kmh']:.0f} km/h")
    
    # Brake
    print("Braking...")
    send_control(brake=True)
    time.sleep(1)
    
    state = get_state()
    print(f"After brake: Speed={state['speed_kmh']:.0f} km/h")
    
    reset_control()
    print("✅ Basic driving test complete\n")


def test_turn_monitoring():
    """Test monitoring a turn for off-road detection."""
    print("\n🔄 Test: Turn Monitoring")
    print("=" * 50)
    
    # Enable breadcrumbs
    print("Enabling breadcrumbs...")
    toggle_breadcrumbs(True)
    
    # Teleport to random location
    print("Teleporting to random location...")
    teleport_random()
    time.sleep(0.5)
    
    state = get_state()
    initial_segment = state['segment']
    print(f"Starting at segment {initial_segment}")
    
    # Accelerate and activate right blinker
    print("Accelerating with right blinker...")
    send_control(accelerate=True, blinker_right=True)
    
    # Monitor for 10 seconds or until segment change
    start_time = time.time()
    max_duration = 10
    off_road_detected = False
    segment_changed = False
    
    while time.time() - start_time < max_duration:
        state = get_state()
        
        if not state['on_road']:
            print(f"❌ OFF-ROAD DETECTED at {state['x']:.0f}, {state['y']:.0f}!")
            off_road_detected = True
            break
        
        if state['segment'] != initial_segment:
            print(f"✅ Segment changed: {initial_segment} → {state['segment']}")
            segment_changed = True
            break
        
        time.sleep(0.1)
    
    reset_control()
    
    if off_road_detected:
        print("❌ Turn test FAILED: Car went off-road")
    elif segment_changed:
        print("✅ Turn test PASSED: Stayed on road")
    else:
        print("⚠️  Turn test TIMEOUT: No segment change in 10s")
    
    print()


def test_screenshot():
    """Test screenshot endpoint."""
    print("\n📷 Test: Screenshot")
    print("=" * 50)
    
    try:
        response = requests.get(f"{API_URL}/screenshot")
        if response.status_code == 200:
            # Save screenshot
            with open('/tmp/api_screenshot.png', 'wb') as f:
                f.write(response.content)
            print("✅ Screenshot saved to /tmp/api_screenshot.png")
        else:
            print(f"❌ Screenshot failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Screenshot error: {e}")
    
    print()


def main():
    """Run all tests."""
    print("\n" + "=" * 50)
    print("REST API Test Suite")
    print("=" * 50)
    
    if not test_health():
        sys.exit(1)
    
    try:
        test_basic_driving()
        test_turn_monitoring()
        test_screenshot()
        
        print("=" * 50)
        print("✅ All tests complete!")
        print("=" * 50)
        print()
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        reset_control()
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        reset_control()


if __name__ == '__main__':
    main()
