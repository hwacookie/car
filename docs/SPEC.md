# Car Game — Specification

## Overview

A 2D top-down driving game rendered with **Pygame**, where the playable world is built from **real road network data extracted from OpenStreetMap (OSM)**. The player drives a car along actual streets in **Kleinmachnow** (south of Berlin).

## Core Concept

- Load the road network from the **OSM-Wars PostgreSQL database** (PostGIS, `road_geometry` table)
- Parse the data into a graph of **nodes** (intersections/points) and **segments** (road pieces)
- Render roads as **2D polygons** with real-world widths (no simple lines)
- Two driving modes: **FREE** (manual steering) and **RAILS** (automatic road following with turn signals)

## Technical Stack

| Layer | Technology |
|-------|-----------|
| Game engine | **Pygame 2.6.1** |
| OSM data source | **OSM-Wars PostgreSQL DB** (PostGIS, `road_geometry` table, Brandenburg schema) |
| DB access | **psycopg3** |
| Language | **Python 3.14** |
| Region | **Kleinmachnow** (south of Berlin) |

## Architecture

```
┌─────────────────────────────────────────────┐
│                  Pygame Window               │
│  ┌─────────────────────────────────────────┐ │
│  │      HUD (bottom-left)                  │ │
│  │      - Speed, Mode, Indicators          │ │
│  └─────────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────┐ │
│  │           Camera / Viewport             │ │
│  │  ┌───────────────────────────────────┐  │ │
│  │  │   Road Network (polygons)         │  │ │
│  │  │   - Direct rendering per frame    │  │ │
│  │  │   - Rounded caps on segments      │  │ │
│  │  └───────────────────────────────────┘  │ │
│  │  ┌───────────────────────────────────┐  │ │
│  │  │         Car Entity                │  │ │
│  │  │   - Headlights (2px)              │  │ │
│  │  │   - Taillights (2px, red)         │  │ │
│  │  │   - Blinkers (3px, orange)        │  │ │
│  │  └───────────────────────────────────┘  │ │
│  └─────────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────┐ │
│  │      Minimap (top-right)                │ │
│  │      - Full area view                   │ │
│  │      - Yellow viewport indicator        │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
         ┌──────────────────────┐
         │  RoadNetwork (graph) │
         │  - nodes + segments  │
         │  - node_degree       │
         │  - node_connections  │
         └──────────────────────┘
```

## Driving Modes

### FREE Mode (Manual Steering)
- **W/↑**: Accelerate
- **S/↓**: Brake (immediate, 10 m/s²)
- **A/←**: Steer left
- **D/→**: Steer right
- **Release W**: Speed maintained (cruise control, no friction)
- **Off-road**: Car stops immediately
- **Map edge**: Car stops at boundary

### RAILS Mode (Automatic Road Following)
- **W/↑**: Accelerate
- **S/↓**: Brake (immediate, 10 m/s²)
- **A/←**: Set **left blinker** → turns left at next junction
- **D/→**: Set **right blinker** → turns right at next junction
- **TAB**: Toggle between FREE ↔ RAILS mode
- **Features**:
  - Car follows road automatically (stays on right lane)
  - Blinker only turns off when **actually turned** in that direction (not at every junction)
  - Automatic braking before sharp curves (physics-based braking distance)
  - "Rechts vor links" logic: only brakes at junctions with roads from the right
  - Dead ends: car turns 180° and stops
  - Smooth segment transitions (no teleportation)

### Mode Switching
- Press **TAB** to toggle between modes
- When switching to RAILS: car snaps to nearest road segment
- Start mode: **RAILS** (automatic)

## Physics

### Speed & Acceleration
- **Max speed**: 180 km/h (50 m/s)
- **Acceleration**: 2.8 m/s² (0–100 km/h in ~10 seconds, normal car)
- **Braking**: 10 m/s² (full ABS braking, ~1g)
- **Cruise control**: W-release maintains speed (no automatic deceleration)

### Automatic Braking (RAILS mode)
- **Braking distance formula**: `s = (v₁² - v₂²) / (2a)`
- **Safe speeds by turn angle**:
  - >90°: 20 km/h (tight turn)
  - >60°: 30 km/h
  - >30°: 40 km/h
  - <30°: no braking (gentle turn)
- **Safety margin**: 5 meters before junction
- **Only at junctions with right-of-way conflict** (road from right, 3+ connections)

### Realistic Turning Physics (RAILS mode)

**Core Rule**: The car must **ALWAYS stay completely on the road** with all four tires. No cutting corners, no off-road driving.

#### Real Car Dynamics
- **Pivot point**: Rear axle (like a real car)
- **Turning radius**: Depends on speed and steering angle
- **Centripetal force**: `F = mv²/r` → faster speed requires wider radius

#### Speed-Based Turning Radius

**Design constraint** (not validated after the fact): Maximum lateral acceleration determines turning radius.

| Speed | Required Turning Radius |
|-------|-------------------------|
| 20-30 km/h | 5-10 m (tight turn) |
| 40-60 km/h | 15-25 m (medium turn) |
| 80+ km/h | 30-50 m (wide turn) |

**Formula**: `radius = k × speed²` (where k is tuned for realistic feel)

**Physics**: Centripetal force `F = mv²/r` means faster speed requires wider radius to keep lateral acceleration within design limits.

**Note**: This is a **design constraint** for the turning system (how we calculate turns), NOT a validation check. External forces (collisions, explosions) CAN exceed this limit—that's physically possible!

#### Geometry-Based Turn Validation

Before starting any turn (at degree-2, degree-3, or degree-3+ nodes):

1. **Calculate required turning radius** based on current speed
2. **Check available road geometry**:
   - Distance remaining on current segment
   - Angle between current and next segment
   - Width of both road segments
   - Calculate if circular arc fits within both segments
3. **Validate arc stays on road**:
   - Start point: X meters before junction (on current road)
   - End point: Y meters into next road
   - Arc must stay within road boundaries at all points
4. **If arc doesn't fit** → brake harder
5. **If cannot brake in time** → **miss the turn**, continue straight

#### Turn Execution

**All nodes** (degree 2, 3, 4+) use the same physics:
- Calculate angle change between segments
- Determine if turn is possible at current speed
- If yes: follow circular arc with calculated radius
- If no: brake or miss turn

**Smooth rotation at degree-2 nodes**:
- Even when "following the road" without changing at an intersection
- Car smoothly rotates to follow road curvature
- No instant heading snaps

**Turn sequence**:
1. **Pre-turn phase** (10-30m before junction):
   - Validate geometry
   - Brake if needed
   - Start rotation early if possible
2. **Arc phase** (through the junction):
   - Follow circular arc with constant radius
   - Heading rotates smoothly
   - Stay within both road segments
3. **Post-turn phase** (settling onto new road):
   - Complete rotation to new road direction
   - Resume normal following

#### Braking Strategy

**Geometry-based braking** (not just angle-based):
```
required_radius = calculate_radius(current_speed)
available_geometry = analyze_roads(current_seg, next_seg, junction)

if arc_fits(required_radius, available_geometry):
    execute_turn()
else:
    brake_harder()
    
if cannot_brake_in_time():
    miss_turn()  # Continue on current road
```

**Missed turn behavior**:
- Blinker stays on
- Car continues on current road (follows "straight")
- Will attempt turn at next junction if blinker still active

### Turning
- **FREE mode**: Turn rate depends on speed (slower at high speed)
- **RAILS mode**: Heading follows road direction automatically

### Teleportation Watchdog
- **Purpose**: Detect unphysical position jumps (bugs)
- **Threshold**: >50 meters per frame
- **Action**: Exception with stack trace + debug info
- **Skip**: First 5 frames (allows initial positioning)

## HUD (Heads-Up Display)

Located in **bottom-left corner**, shows:

### Speed Display
- **Large number**: Current speed in km/h
- **Color coding**:
  - White: 0–50 km/h
  - Yellow: 50–100 km/h
  - Red: 100+ km/h
- **Unit**: "km/h" label
- **Speedometer arc**: 0–180 km/h circular gauge (green → yellow → red)

### Mode Indicator
- **Text**: "FREE" or "RAILS"
- **Color**: Blue (FREE) or Green (RAILS)
- **Hint**: "(TAB)" to switch

### Status Indicators
- **B** (red circle): Braking active
- **A** (green circle): Accelerating
- **L** (orange circle): Left blinker on
- **R** (orange circle): Right blinker on

## Road Network

### Data Source
- **Database**: OSM-Wars PostgreSQL, schema `brandenburg`
- **Table**: `road_geometry` (EPSG:3857 projected coordinates)
- **Area**: Kleinmachnow bounding box
  - North: 52.42382°, West: 13.21831°
  - South: 52.40714°, East: 13.25033°
- **Segments loaded**: 1970 road segments
- **Nodes**: 1909 unique nodes

### Drivable Road Types (highway_id filter)
Only roads with these highway types are loaded:

| highway_id | OSM tag | Description |
|------------|---------|-------------|
| 1 | motorway | Highway/Autobahn |
| 2 | trunk | Major trunk road |
| 3 | primary | Primary road |
| 4 | secondary | Secondary road |
| 5 | tertiary | Tertiary road |
| 6 | residential | Residential street |
| 7 | service | Service road/driveway |
| 8 | unclassified | Unclassified road |
| 9 | motorway_link | Motorway ramp |
| 10 | trunk_link | Trunk ramp |
| 11 | primary_link | Primary ramp |
| 12 | secondary_link | Secondary ramp |
| 14 | tertiary_link | Tertiary ramp |

### Road Widths (meters)

| Highway Type | 2-way width | 1-way width | Color |
|--------------|-------------|-------------|-------|
| motorway | 14.0 m | 7.0 m | #444444 |
| trunk | 10.0 m | 7.0 m | #555555 |
| primary | 10.0 m | 7.0 m | #666666 |
| secondary | 7.0 m | 3.5 m | #888888 |
| tertiary | 7.0 m | 3.5 m | #999999 |
| residential | 7.0 m | 3.5 m | #aaaaaa |
| service | 3.5 m | 3.5 m | #cccccc |

**Special case**: Service roads always use 1-way width (3.5 m) regardless of `oneway` tag.

### Rendering
- **Style**: Direct polygon rendering (no texture scaling)
- **Geometry**: Rectangle + rounded caps per segment
- **Per frame**: Roads drawn as vector polygons (no pixelation on zoom)
- **Performance**: 109 FPS with 1970 segments at 4× zoom

### Graph Structure
- **Segments**: List of `RoadSegment` objects with:
  - Coordinates (x1, y1, x2, y2)
  - Highway type, oneway flag, width
  - Start/end node IDs
  - Length in meters
- **Nodes**: Dict of node_id → (x, y) positions
- **Node connections**: Dict of node_id → [segment indices]
- **Node degree**: Dict of node_id → connection count (for junction detection)

### Endpoint Snapping
- **Threshold**: 8 meters
- **Purpose**: Close OSM mapping gaps
- **Result**: 73 endpoints snapped in Kleinmachnow

## Camera

### Controls
- **Mouse wheel**: Zoom in/out
- **Middle mouse button + drag**: Pan map manually
- **+/- keys**: Zoom in/out
- **C key**: Snap camera to car position

### Behavior
- **Follow mode**: Camera follows car when `speed > 0.1 m/s`
- **Zoom range**: 0.64× to 16× (configurable)
- **Smooth follow**: Interpolated camera movement
- **Manual override**: Middle-mouse drag disables follow temporarily

## Minimap

- **Location**: Top-right corner
- **Size**: 200×200 px
- **Content**: 
  - Full road network (scaled down)
  - Car position (blue dot)
  - Current viewport (yellow rectangle, Y-axis aligned with main view)
  - Speed text overlay

## Car Visual

### Body
- **Size**: 4.5 m × 2.0 m (length × width)
- **Color**: Red (#B41E1E body, #D73C3C front strip)
- **Rotation**: Smooth heading (0° = north/up)

### Lights
- **Headlights**: 2× white circles (2px) at front corners
- **Taillights**: 2× red circles (2px) at rear corners
  - Bright red when braking
  - Dark red otherwise
- **Blinkers** (RAILS mode only): 3× orange circles (3px) at side
  - Flash with 0.5s period (on 0.25s, off 0.25s)
  - Left or right depending on signal

## Coordinate System

- **World space**: EPSG:3857 projection (meters)
- **Screen space**: Pixels
- **Y-axis**: 
  - World: Y increases northward
  - Screen: Y increases downward
  - Camera handles transform
- **PIXELS_PER_METER**: 10 (configurable)

## Configuration (`config.py`)

### Window
```python
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
BG_COLOR = (34, 34, 34)
```

### Car Physics
```python
CAR_SPEED = 50          # m/s = 180 km/h
CAR_ACCELERATION = 2.8  # m/s² (0-100 in ~10s)
CAR_BRAKING = 10.0      # m/s² (ABS braking)
CAR_TURN_SPEED = 180    # degrees/second
CAR_LENGTH = 4.5        # meters
CAR_WIDTH = 2.0         # meters
```

### Minimap
```python
MINIMAP_SIZE = 200
MINIMAP_MARGIN = 10
MINIMAP_BG = (20, 20, 20, 150)
MINIMAP_BORDER = (100, 100, 100)
MINIMAP_CAR_COLOR = (100, 150, 255)
```

### Bounding Box (Kleinmachnow)
```python
BOUNDING_BOX = {
    "north": 52.42382,
    "west": 13.21831,
    "south": 52.40714,
    "east": 13.25033,
}
```

## Project Structure

```
car/
├── docs/
│   └── SPEC.md              # This file
├── src/
│   ├── __init__.py
│   ├── main.py              # Entry point, game loop
│   ├── config.py            # Settings & constants
│   ├── osm_loader.py        # PostgreSQL data fetcher
│   ├── road_network.py      # Graph + spatial queries
│   ├── renderer.py          # Pygame drawing (roads, HUD, minimap)
│   ├── car.py               # Car physics & modes
│   └── camera.py            # Camera / viewport logic
├── tests/
│   ├── __init__.py
│   └── test_road_network.py  # 10 unit tests (projection, spatial)
├── requirements.txt
├── .gitignore
└── README.md
```

## Implementation Status

### ✅ Completed Features
- [x] PostgreSQL OSM data loader (Brandenburg schema)
- [x] Road network graph with node degrees & connections
- [x] Direct polygon rendering (no pixelation on zoom)
- [x] Car physics with two modes (FREE/RAILS)
- [x] Automatic road following (RAILS mode)
- [x] Turn signals & intelligent blinker logic
- [x] Automatic braking with physics-based distance calculation
- [x] "Rechts vor links" junction logic
- [x] HUD with speed, mode, and status indicators
- [x] Minimap with viewport indicator
- [x] Camera follow & manual pan/zoom
- [x] Headlights & taillights (brake lights)
- [x] Blinker visualization
- [x] Dead-end handling (180° turn)
- [x] Teleportation watchdog (bug detection)
- [x] Service road width (3.5m fixed)
- [x] Endpoint snapping (8m threshold)
- [x] Off-road detection (FREE mode)
- [x] Breadcrumb trail (cyan dots showing driven path)
- [x] 10 unit tests passing

### 🚧 In Progress
- [ ] **Realistic turning physics** (circular arc, geometry-based)
  - Speed-dependent turning radius
  - Geometry validation (arc must fit within road)
  - Smooth rotation at all nodes (degree 2, 3, 4+)
  - Miss turn if cannot brake in time
  - Always stay on road (strict constraint)

### 🐛 Known Issues
- **RAILS mode turns go off-road**: Current Bezier curve implementation cuts corners
- **Instant heading changes**: At degree-2 nodes, heading snaps instead of smooth rotation
- **Unrealistic braking**: Brakes by angle, not by geometry constraints

### 🔮 Future Enhancements
- Building footprints from OSM `building` polygons
- Road surface textures (asphalt, cobblestone)
- Road markings (center lines, crosswalks)
- Traffic signs
- Day/night cycle with dynamic lighting
- Multiple cars / traffic simulation
- Sound effects (engine, brakes)
- Anti-aliasing via pygame.gfxdraw

## Performance

- **FPS**: 109 fps @ 4× zoom with 1970 segments
- **Rendering**: Vector polygons drawn per frame
- **Database query**: <1 second for Kleinmachnow area
- **Memory**: ~50 MB for full network

## Testing

### Unit Tests
```bash
python -m pytest tests/
```

### Smoke Test
```bash
python -m src.main --smoke 60  # Run 60 frames headless
```

### Manual Testing
```bash
python -m src.main
```

## Controls Summary

| Key | FREE Mode | RAILS Mode |
|-----|-----------|------------|
| W/↑ | Accelerate | Accelerate |
| S/↓ | Brake | Brake |
| A/← | Steer left | Left blinker → turn left at junction |
| D/→ | Steer right | Right blinker → turn right at junction |
| TAB | → RAILS mode | → FREE mode |
| C | Snap camera to car | Snap camera to car |
| B | Toggle breadcrumb trail | Toggle breadcrumb trail |
| R | Random location (teleport) | Random location (teleport) |
| V | Toggle physics validator | Toggle physics validator |
| +/- | Zoom in/out | Zoom in/out |
| Scroll | Zoom in/out | Zoom in/out |
| Middle mouse | Pan map | Pan map |

## Game Loop (60 FPS)

1. **Input**: Process keyboard & mouse events
2. **Physics**: Update car position, speed, heading (mode-dependent)
3. **Collisions**: Check off-road, map edges (FREE mode only)
4. **Watchdog**: Detect teleportation (>50m jumps)
5. **Camera**: Update viewport (smooth follow if moving)
6. **Render**:
   - Roads (direct polygon drawing)
   - Car (sprite + lights + blinkers)
   - HUD (speed, mode, indicators)
   - Minimap
7. **Display**: Flip buffer (vsync at 60 Hz)

## Physics Validator ("Physics Judge")

**Independent validation system** that runs separately from car physics to detect **hard physics constraint violations**.

### Philosophy: Hard Invariants Only

PhysicsValidator checks **only constraints that can NEVER be violated** by the laws of physics:

✅ **Checked** (always impossible):
- Position discontinuity (teleportation)
- Rotation discontinuity (instant heading snap)
- Solid object overlap (collisions)
- Going through solid boundaries (off-road)

❌ **NOT Checked** (can be exceeded by external forces):
- Maximum lateral acceleration (lorry crash can exceed this!)
- Maximum speed (external forces can push car faster)
- Speed limits (traffic rule, not physics)
- Lane discipline (design preference, not physics law)

**Key insight**: If a lorry crashes into a car sideways, the car CAN experience huge lateral acceleration. This is physically possible! So it's NOT a validation check—it's a **design constraint** used during turning calculation.

### Architecture
- **Separate class** (`PhysicsValidator`) - not embedded in `Car`
- **Toggleable** with V key (enabled by default during development)
- **Per-car state tracking** - supports multiple cars
- **Performance-friendly** - disable for proven-good cars

### Current Checks
1. **Teleportation detection**: Position jumps > 50m
2. **Instant heading changes**: Rotations > 30° in one frame (RAILS mode)
3. **Off-road violations**: Car leaving road in RAILS mode
4. **Collision detection**: (TODO) Two cars occupying same space

### Future: TrafficPolice Class

For **traffic rules** (not physics), we'll create a separate `TrafficPolice` class:
- Speed limit violations 🚔
- Red light running 🚦
- Stop sign violations 🛑
- One-way street violations ➡️
- Lane discipline checks

These are **legal constraints**, not physics constraints.

### Usage Pattern
```python
validator = PhysicsValidator(enabled=True)

# Game loop:
car.update(dt, network, control_input)
validator.check(car, dt, network)  # Independent check

# After intentional teleport:
car.teleport_random(network)
validator.reset_car_state(car)  # Skip next 5 frames
```

### Benefits
- ✅ Separation of concerns (physics vs validation)
- ✅ Can be disabled for performance
- ✅ Re-enable when experimenting with new features
- ✅ Works with multiple cars
- ✅ Easy to extend with new checks

## Debug Features

### Teleportation Watchdog Output
```
======================================================================
TELEPORTATION DETECTED!
======================================================================
Old position: (1280.9, 1566.7)
New position: (4351.6, 3267.1)
Distance: 1755.0m (max allowed: 50.0m)
Speed: 0.0 m/s (0 km/h)
Mode: rails
Segment: 0, Progress: 0.500, Forward: True
dt: 0.0170s
======================================================================
[Stack trace follows]
```

### Frame Dump
```bash
python -m src.main --dump  # Saves frame 30 to /tmp/car_frame.bmp
```

## Technical Decisions

### Why Direct Polygon Rendering?
- **No pixelation** on zoom (vector vs raster)
- Real-time width scaling with zoom
- Smooth rounded caps on segments
- 109 FPS with 1970 segments (fast enough)

### Why Two Driving Modes?
- **FREE**: Classic driving game feel, full control
- **RAILS**: Relaxed driving, focus on route planning, realistic turn signals

### Why Teleportation Watchdog?
- Catches segment transition bugs early
- Provides detailed debug info
- Prevents game-breaking jumps

### Why "Rechts vor links" Logic?
- Realistic German traffic rules
- Reduces unnecessary braking (only at conflict junctions)
- Makes automatic driving smoother

---

**Last Updated**: 2026-01-10  
**Status**: 🚧 In active development - implementing realistic turning physics  
**Version**: 0.9 (playable, physics improvements in progress)
