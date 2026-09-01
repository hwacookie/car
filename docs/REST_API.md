# REST API Documentation

## Overview

The game includes an optional REST API for remote control and automated testing. This allows you to:
- Control the car programmatically
- Monitor game state in real-time
- Automate testing scenarios
- Freeze / resume the simulation
- Toggle features remotely

## Starting the API

Run the game with the `--api` flag:

```bash
python -m src.main --api
```

The API server will start on `http://localhost:5000`

## Endpoints

### Health Check
```bash
GET /health
```

Returns:
```json
{
  "status": "ok",
  "timestamp": 1234567890.123
}
```

### Get Game State
```bash
GET /state
```

Returns current game state:
```json
{
  "frame": 123,
  "time": 2.05,
  "x": 1234.5,
  "y": 2345.6,
  "heading": 45.0,
  "speed": 10.5,
  "speed_kmh": 37.8,
  "segment": 42,
  "progress": 0.65,
  "forward": true,
  "on_road": true,
  "driver": "BICYCLE",
  "parking": {
    "style": "reverse",
    "phase": "stopped",
    "parked": true,
    "reversing": false
  },
  "trail_enabled": false,
  "validator_enabled": true,
  "camera_x": 1234.5,
  "camera_y": 2345.6,
  "camera_zoom": 2.0
}
```

`parking` reflects the nav's parking state (fields are null when no
parking plan is active): `style` is `"forward"` or `"reverse"`, `phase`
tracks the approach (`lead/decel/swerve/final/reverse/stopped`), and
`parked` becomes `true` only when the maneuver is complete - this is what
the e2e suite waits for on reverse-in scenarios, because a back-in park
deliberately crosses the end flag to stage the reverse (see
`docs/TESTING.md` §3).

### Send Control Inputs
```bash
POST /control
Content-Type: application/json

{
  "accelerate": true,
  "brake": false,
  "steer_left": false,
  "steer_right": false,
  "blinker_left": false,
  "blinker_right": true
}
```

All fields are optional. Only specified fields are updated.

### Reset Controls
```bash
POST /reset
```

Sets all control inputs to `false`.

### Teleport Car
```bash
POST /teleport
Content-Type: application/json

# Random location
{"random": true}

# Specific location (TODO)
{"segment": 42, "progress": 0.5}
```

### Set HUD Label
```bash
POST /label
Content-Type: application/json

{"text": "4"}
```
Shows (or, with `{"text": null}` / `{}`, clears) a short text label in
the game's HUD, just below the minimap. Purely a visual aid - e.g.
`tests/test_turning.py` uses it to show which map tile/scenario the
currently running test is on.

### Toggle Features
```bash
POST /toggle
Content-Type: application/json

{
  "breadcrumbs": true,
  "validator": false,
  "mode": "bicycle"  # or "free"
}
```

### Freeze / Resume
```bash
POST /freeze
Content-Type: application/json

{"frozen": true}   # or false to resume
```

Pauses the simulation (replaces the old ESC key in the pygame window).
`GET /state` reports `"frozen": true` while paused.

### Wait for Condition
```bash
POST /wait
Content-Type: application/json

{
  "condition": "segment_changed",  # or "speed_reached", "position_reached"
  "value": 42,  # depends on condition
  "timeout": 5.0  # seconds
}
```

Blocks until condition is met or timeout.

### Obstacles (docs/OBSTACLES.md)

The game has a static obstacle system (parked cars). Placement and removal
go through the same logic as the palette UI in the game window.

**List obstacles**
```bash
GET /obstacles
```
Returns `[{id, type, color, x, y, heading}, ...]` — `x`/`y` in world pixels
(2 px per meter), `heading` in degrees (0 = north). Empty list if none are
placed.

**Place an obstacle**
```bash
POST /obstacles
Content-Type: application/json

{"type": "car", "color": "blue", "x": 194, "y": 300}
```
- `type`: currently only `"car"` (a parked car, same size as the player car)
- `color`: `blue`, `yellow` or `white`
- `x`, `y`: world pixels (2 px per meter — divide by 2 for meters)

The heading is NOT an input: it is computed from the lane direction under the
drop point (right half of the road faces forward, left/oncoming half faces
back; on curves and in junctions it follows the local road tangent).
Returns `201` + the created obstacle (with its stable `id`). Returns `400`
if the point is off the paved area or the request is invalid.

**Remove an obstacle**
```bash
DELETE /obstacles/1
```
Returns `200` (`{"ok": true, "id": 1}`) or `404` for an unknown id.

## Example Usage

### Python (requests)

```python
import requests
import time

API = "http://localhost:5000"

# Check health
response = requests.get(f"{API}/health")
print(response.json())

# Get state
state = requests.get(f"{API}/state").json()
print(f"Speed: {state['speed_kmh']:.0f} km/h")

# Accelerate
requests.post(f"{API}/control", json={"accelerate": True})
time.sleep(2)

# Check speed again
state = requests.get(f"{API}/state").json()
print(f"New speed: {state['speed_kmh']:.0f} km/h")

# Stop
requests.post(f"{API}/reset")
```

### curl

```bash
# Health check
curl http://localhost:5000/health

# Get state
curl http://localhost:5000/state | jq

# Accelerate
curl -X POST http://localhost:5000/control \
  -H "Content-Type: application/json" \
  -d '{"accelerate": true}'

# Teleport randomly
curl -X POST http://localhost:5000/teleport \
  -H "Content-Type: application/json" \
  -d '{"random": true}'

# Enable breadcrumbs
curl -X POST http://localhost:5000/toggle \
  -H "Content-Type: application/json" \
  -d '{"breadcrumbs": true}'

# Freeze the simulation (e.g. to inspect state)
curl -X POST http://localhost:5000/freeze \
  -H "Content-Type: application/json" -d '{"frozen": true}'
```

## Automated Testing

See `tests/test_api.py` for examples of automated tests:

```bash
# Terminal 1: Start game with API
python -m src.main --api

# Terminal 2: Run tests
python tests/test_api.py
```

The test script demonstrates:
- Basic driving (accelerate, brake, coast)
- Turn monitoring (detect off-road)
- Screenshot capture

## Use Cases

### 1. Automated Turn Testing

```python
def test_turn_stays_on_road():
    # Teleport to junction
    requests.post(f"{API}/teleport", json={"random": True})
    
    # Accelerate with right blinker
    requests.post(f"{API}/control", json={
        "accelerate": True,
        "blinker_right": True
    })
    
    # Monitor for 10 seconds
    for i in range(100):
        state = requests.get(f"{API}/state").json()
        assert state['on_road'], "Car went off-road!"
        time.sleep(0.1)
```

### 2. Performance Profiling

```python
states = []
for i in range(600):  # 10 seconds at 60 FPS
    state = requests.get(f"{API}/state").json()
    states.append(state)
    time.sleep(1/60)

# Analyze positions, speeds, etc.
```

### 3. State Snapshots

Since M5 the sim is headless - there is no server-side rendering to
screenshot. Visual verification happens in the Godot frontend; for
automated checks, snapshot `GET /state` (position, heading, segment,
flags) instead of pixels.

## Architecture

The API runs in a **separate thread** from the game loop:
- **Thread-safe** state sharing via `threading.Lock`
- **Non-blocking** - game continues at 60 FPS (headless, self-paced)

Game loop flow:
1. Handle API commands (teleport, toggle, freeze, flags, label)
2. Update car physics (fixed timestep, accumulator)
3. Update camera
4. Update API state

## Security Note

The API listens on `127.0.0.1:5000` (localhost only) by default.
**Do not expose to public networks** - there is no authentication!
