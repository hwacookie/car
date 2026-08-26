# Obstacles — Specification

Foundation for the **Ausweichen** maneuver (`DRIVING_MANEUVERS.md` §6):
before avoidance behavior can be specified and tested, obstacles must be
placeable on the road. This document covers **Part 1: placement tooling +
static car obstacles**. The avoidance behavior itself is NOT part of this —
it remains to be specified in `DRIVING_MANEUVERS.md` §6.

## World Model

- An **obstacle** is a world object with position `(x, y)`, heading, type and
  color. It is drawn into the scene and exists until deleted.
- Obstacles come in two **classes**:
  - **Static** (implemented in Part 1): never moves. Current type: the
    *parked car* — same size as the player car (`config.CAR_LENGTH` ×
    `CAR_WIDTH`, 4.4 m × 1.8 m), same sprite asset, recolored. It is scenery that other cars must deal with
    (`DRIVING_MANEUVERS.md` §6).
  - **Movable** (later parts, not yet specified): other cars, pedestrians,
    children. They move — they may leave the collision point before the car
    arrives, and may only be visible from far away — so avoiding them requires
    **prediction** of future collisions, not just current geometry. Their
    behavior is out of scope here; static avoidance is specified now.
- The palette offers three colors for the parked car: blue, yellow, white
  (concrete RGB values are a rendering detail; they must stay clearly
  distinguishable from the player's red `#B41E1E` and from each other).
- **Placement constraint**: an obstacle may only be placed on the paved area
  — the same Shapely paved polygon that is the single source of truth for the
  on-road check (`RoadNetwork.is_on_road`). A drop off the road is rejected.
- **Auto-alignment** (explicit requirement): when an obstacle is dropped on a
  street it aligns with the road automatically:
  - **Heading** = direction of travel of the lane under the drop point: a car
    dropped in the right half faces forward, one dropped in the left/oncoming
    half faces the other way — like a car stopped in traffic in that lane.
    (Decision 2026-08-25.)
  - **Lateral position is free**: the obstacle is placed exactly where it was
    dropped — anywhere on the paved area, no snapping to a lane centre. This
    allows setups like a car blocking only half a lane.
    (Decision 2026-08-25.)
  - **Curves and junctions**: the heading is the local tangent of the
    *smoothed* centerline — the Catmull-Rom spline / junction fillet arc that
    defines the pavement (SPEC.md §10) — not the chord of a straight segment.
    A car dropped on a V-shaped bend points along the road at that spot, and
    one dropped in an intersection aligns with the diagonal through the
    corner. What is drawn is exactly what is driven.
- The alignment is computed at drop time from the road geometry; obstacles do
  not re-align later (the road does not move).
- Every obstacle has a **stable id** (assigned on placement) — required for
  removal via the REST API and unambiguous in saved layouts.

## Palette UI (the box next to the minimap)

A fixed panel in the top-right corner, **immediately left of the minimap**
(the HUD stays bottom-left; the palette must not overlap either).

```
┌───────────────┬────────────┐
│  OBSTACLES    │  MINIMAP   │
│  ┌──┐ ┌──┐ ┌──┐│            │
│  │🚗│ │🚗│ │🚗││            │
│  │blu│ │yel│ │wht││            │
│  └──┘ └──┘ └──┘│            │
│  [SAVE] [LOAD] │            │
│      [ 🗑 ]    │            │
└───────────────┴────────────┘
```

- **Three slots**, one static car per color (blue / yellow / white).
- **SAVE / LOAD** buttons (see "Save / Load" below).
- **Trashcan** at the bottom of the box.

### Interactions (left mouse button, shared with map panning: a LMB
hold-drag on empty map area pans the camera; every press that starts a
palette drag - a slot or a world obstacle - is consumed by the UI)

1. **Place**: press on a palette slot → a ghost car follows the cursor over
   the map, already shown with its final heading (lane travel direction under
   the cursor, live-updating while dragging); laterally it sits exactly where
   the cursor is. Release over the paved area → obstacle is placed there.
   Release off-road or back over the palette → cancelled, nothing is placed.
2. **Move**: press on an obstacle that is already in the world → it is picked
   up (same ghost behavior as above). Release over the paved area → it is
   re-placed at the new position and re-aligned there.
3. **Delete**: drag a world obstacle onto the trashcan → release → the
   obstacle is removed from the world. The trashcan highlights while a
   dragged obstacle hovers over it. (Dropping a fresh palette item into the
   trashcan is simply a cancel — no-op.)

While dragging, the ghost is drawn at full size with reduced opacity; an
invalid drop target (off-road) is signalled by tinting the ghost red so the
rejection on release is never surprising.

## Save / Load (Obstacle Layouts)

The road network itself is never saved — it comes from the OSM database or
the test-map builder. What is saved is the **obstacle layout** on top of it,
so that a scenario can be set up once and driven again later (and shared).

- A **layout** = the current set of obstacles: for each one its type, color,
  world coordinates `(x, y)` and heading. Nothing else (car position, camera)
is part of a layout.
- Layouts are stored as human-readable **JSON files**, one file per named
  layout, under `data/obstacles/<map_name>/` (alongside the existing
  `data/osm_cache/`). The map name is recorded inside the file; LOAD only
  offers layouts that belong to the **current** map — a Kleinmachnow layout
  must not be loadable onto the synthetic `basic` map.
- **SAVE**: stores the current set under a name (typed into a small input
  field in the window); an existing layout of the same name is overwritten.
- **LOAD**: shows the saved layouts of the current map as clickable entries
  in the palette box; selecting one **replaces** the current obstacles with
  the loaded set.
- **Validation on load**: each entry is checked against the paved polygon;
  an obstacle that no longer lies on the road (e.g. the map data changed)
is skipped with a warning instead of being placed off-road.

## REST API (placing / removing obstacles)

The palette is the manual tool; the REST API is the programmatic path —
required so obstacle scenarios can be scripted into the deterministic e2e
suite (decision 2026-08-25). Both paths go through the **same** placement
logic: identical auto-alignment, identical off-road rejection.

```
GET    /obstacles            → list of placed obstacles
                                 [{id, type, color, x, y, heading}, ...]
POST   /obstacles            → {"type": "car", "color": "blue|yellow|white",
                                "x": <m>, "y": <m>}
                                → 201 + created obstacle (id + computed
                                  heading); 4xx if the point is off-road
DELETE /obstacles/<id>       → removes the obstacle; 404 if unknown id
```

- `x`/`y` are world coordinates, same as in saved layouts.
- Heading is never a client input — it is always computed from the road
  geometry at the drop point (same rule as the palette).
- Obstacles placed via API and via palette are the same world objects: both
  appear in `GET /obstacles`, both can be dragged/trashed in the UI, both are
  saved by SAVE.

## Stop on Contact (collision with obstacles)

The player car **stops when it touches an obstacle** — it never passes
through one (decision 2026-08-25). Implemented as a per-frame geometric check
in the same pattern as the off-road detector:

- **Detection**: every frame, test the player car's body box (the same
  four-corner geometry the on-road check uses) against each obstacle's
  footprint (for now: 4.5 m × 2.0 m rectangle at its position and heading;
  applies to both classes — a movable obstacle is checked where it is *now*).
  Intersection = contact.
- **Response — physically plausible, per the hard rule in AGENTS.md**:
  - On first contact the car brakes with full braking deceleration
    (`A_BRAKE`) until stopped — no instant velocity zeroing from speed.
  - While in contact the car does not move into the obstacle: forward motion
    is clamped so the two boxes never interpenetrate (the car rests against
    it, like against a wall).
- **Applies to all modes** (FREE and BICYCLE), same as the off-road check.
- A contact stop is **expected behavior, not a violation**: it must not trip
  the PhysicsValidator or count as a test failure. The validator keeps
  checking that the stop itself was physical (no penetration, no teleport,
  no instant snap).
- Until §6 avoidance is implemented, this is the safety net: the car drives
  straight and stops at the obstacle. §6 will make it avoid earlier; the
  contact stop remains as the last line behind it.

## Rendering

- Obstacles are drawn like the player car: same sprite pipeline (scale with
  zoom, rotate to heading), above road markings and below HUD/minimap/palette.
- They do not appear on the minimap (for now).

## Explicitly Out of Scope (Part 1)

- **Movable obstacles** — class defined above; their avoidance
  (prediction-based) is a later part. Static avoidance is specified now in
  `DRIVING_MANEUVERS.md` §6.
- No other static obstacle types (cones, trucks, construction sites).

## Decisions (2026-08-25, binding)

1. **Free lateral placement** — no lane-centre snap; the obstacle goes where
   it is dropped (heading still auto-aligned).
2. **Orientation = lane travel direction** of the lane under the drop point
   (oncoming half faces the other way).
3. **REST API for placing/removing obstacles is in scope** (see section above),
   not a later addition.
4. **Stop on contact** — the player car stops at an obstacle, implemented as a
   per-frame geometric check like the off-road detector; physically plausible
   braking, no interpenetration.
5. **Two obstacle classes: static and movable.** Part 1 = static only.
   Static avoidance is specified now (`DRIVING_MANEUVERS.md` §6); movable
   obstacles (other cars, pedestrians, children — prediction-based) come later.

## Cross-References

- `DRIVING_MANEUVERS.md` §6 — Ausweichen: the behavior spec that consumes obstacles
- `SPEC.md` — paved polygon / on-road check (shared geometry), car sprite,
  Visual Layout, controls
