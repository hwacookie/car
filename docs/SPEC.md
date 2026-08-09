# Car Game — Specification

## Overview

A 2D top-down driving game rendered with **Pygame**, where the playable world is built from **real road network data extracted from OpenStreetMap (OSM)**. The player drives a car along actual streets.

## Core Concept

- Download a region's road network from OSM via the **Overpass API**
- Parse the data into a graph of **nodes** (intersections/points) and **ways** (road segments)
- Render the road network as a 2D map in Pygame
- Place a controllable car that can drive along the roads

## Technical Stack

| Layer | Technology |
|-------|-----------|
| Game engine | **Pygame** |
| OSM data fetching | **overpass** (Python library) or direct Overpass API HTTP calls |
| OSM parsing | **ox** (OSMnx) or **lxml** / raw XML/JSON parsing |
| Language | **Python 3** |

## Architecture

```
┌─────────────────────────────────────────────┐
│                  Pygame Window               │
│  ┌─────────────────────────────────────────┐ │
│  │           Camera / Viewport             │ │
│  │  ┌───────────────────────────────────┐  │ │
│  │  │       Road Network Renderer       │  │ │
│  │  │  (draws lines, colors by type)    │  │ │
│  │  └───────────────────────────────────┘  │ │
│  │  ┌───────────────────────────────────┐  │ │
│  │  │         Car Entity                │  │ │
│  │  │  (position, speed, heading)       │  │ │
│  │  └───────────────────────────────────┘  │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
         ┌──────────────────────┐
         │  RoadNetwork (graph) │
         │  nodes + edges       │
         └──────────────────────┘
         ┌──────────────────────┐
         │   OSM Data Loader    │
         │   Overpass API       │
         └──────────────────────┘
```

## Data Flow

1. **Load**: User picks a location (lat/lon + radius or bounding box)
2. **Fetch**: Query Overpass API for `highway=*` ways + nodes in that area
3. **Parse**: Build an internal graph — list of road segments with coordinates
4. **Transform**: Project lat/lon → pixel coordinates (simple equirectangular or mercator)
5. **Render**: Draw each segment as a colored line; line width varies by road type

## Key Components

### 1. OSM Data Loader (`osm_loader.py`)
- Query Overpass API for a bounding box
- Filter for road elements (`highway=*`)
- Return structured data: nodes `{id: (lat, lon)}` and ways `[node_id, ...]`

### 2. Road Network (`road_network.py`)
- Stores the graph of roads
- Projects geographic coordinates to screen pixels
- Exposes methods like `get_road_at(position)` for collision/checking

### 3. Renderer (`renderer.py`)
- Draws the road network onto a Pygame surface
- Color-codes by road type (motorway = dark, residential = light, etc.)
- Supports camera panning (follows the car)

### 4. Car (`car.py`)
- Position (x, y), heading (angle), speed
- Controls: arrow keys or WASD
- Acceleration, deceleration, turning mechanics
- Optional: constrained to roads or free driving

## Road Types & Visual Mapping

Roads are drawn with **real-world widths** (in meters). Zoom controls how many pixels those meters occupy.

### Lane model
- **Lane width**: 3.5 m
- **2-way road**: 2 lanes = 7 m minimum width
- **1-way road**: 1 lane = 3.5 m width
- OSM `oneway` tags determine lane count

### Width by road type (real-world meters)

| OSM highway tag | Color | Width (2-way) | Width (1-way) |
|-----------------|-------|---------------|---------------|
| motorway | #444444 (dark gray) | 14 m (4 lanes) | 7 m |
| trunk | #555555 | 10 m (3 lanes) | 7 m |
| primary | #666666 | 10 m (3 lanes) | 7 m |
| secondary | #888888 | 7 m (2 lanes) | 3.5 m |
| tertiary | #999999 | 7 m (2 lanes) | 3.5 m |
| residential | #aaaaaa | 7 m (2 lanes) | 3.5 m |
| unclassified | #bbbbbb | 7 m (2 lanes) | 3.5 m |
| service | #cccccc | 7 m (2 lanes) | 3.5 m |

### Zoom range
- **Max zoom-in**: ~40 m × 40 m viewport
- **Max zoom-out**: ~1000 m × 1000 m viewport
- Zoom via scroll wheel and/or +/- keys

## Milestones

### Phase 1: Static Map Rendering
- [ ] Fetch OSM data for the target bounding box
- [ ] Parse nodes + ways into an in-memory graph (drivable roads only)
- [ ] Project coordinates and draw the road network in Pygame
- [ ] Color-code + width-map roads by type
- [ ] Minimap showing full area

### Phase 2: Car Movement
- [ ] Car sprite with front/rear, headlights, taillights (brake lights)
- [ ] Keyboard controls (WASD + arrows): accelerate, brake, steer
- [ ] Smooth turning physics
- [ ] Camera follows the car (smooth follow)
- [ ] Zoom in/out (scroll wheel or +/- keys)
- [ ] Place car on random road at start

### Phase 3: Road Conformance
- [ ] Constrain car to roads — off-road = car stops
- [ ] Respect one-way streets
- [ ] Map edge = invisible wall / crash

### Phase 4: Buildings & Polish (Future)
- [ ] Draw building footprints (from OSM `building` tags)
- [ ] Background terrain color (parks, water)
- [ ] Save/load regions
- [ ] Day/night cycle (headlights more visible at night)

## Project Structure

```
car/
├── SPEC.md
├── main.py              # Entry point
├── osm_loader.py        # Overpass API queries
├── road_network.py      # Graph data structure + projection
├── renderer.py          # Pygame drawing
├── car.py               # Car entity + physics
├── config.py            # Settings (colors, speeds, etc.)
├── requirements.txt
└── assets/              # Images, fonts
```

## Decisions

### Target Area

Bounding box (Kleinmachnow, south of Berlin):
- Upper-left:  `52.42382, 13.21831`
- Lower-right: `52.40714, 13.25033`

### Car
- **Controls**: Both arrow keys **and** WASD
- **Start position**: Placed on a random road at game start
- **Appearance**: Sprite with clear front/rear distinction, **headlights**, and **taillights** that illuminate when braking
- **Turning**: Smooth rotation (not snapped)
- **Turning at intersections**: Free turning (align naturally with momentum)

### Roads
- **One-way streets**: Respect OSM `oneway` tags — car cannot drive against one-way
- **Road types**: Only drivable roads (motorway, trunk, primary, secondary, tertiary, residential, unclassified, service)
- **Exclude**: Footways, cycleways, paths, pedestrian zones, etc.
- **Map edges**: Invisible wall — trying to cross = crash/stop

### Visual
- **View**: Pure top-down 2D
- **Camera**: Follows the car (smooth follow, centered on car)
- **Zoom**: Zoom in/out ability (scroll wheel or +/- keys)
- **Road width**: Mapped from highway tag type
- **Background**: Plain base color for Phase 1; **building footprints** as drawn shapes planned for Phase 3

### Gameplay
- **Mode**: Free driving — no objectives, no win/lose
- **Off-road**: Car stops immediately if it leaves the road
- **UI**: **Minimap** in corner showing full area + car position
- **Architecture**: Clear module separation:
  - OSM data loader
  - Road network / world
  - Car movement / input
  - Camera / viewport (smooth follow + zoom)
  - Renderer
  - Game state
