# Godot Frontend — Architecture & Migration Plan

> Status: agreed design decisions (2026-08-31), no implementation yet.
> Companion to `REST_API.md` (the API this document turns into a product
> boundary) and `SMOOTH_GEOMETRY_DESIGN.md` (the geometry contract).

## 1. Goal

Replace the pygame frontend with a Godot 4 frontend (project:
`../driving-game/`, Godot 4.7, C#): faster rendering, and a path to
oblique/3D presentation later. The Python simulation — from now on called
the **world simulator** — stays the single source of truth for physics and
world rules. Nothing in this plan changes simulated behavior; e2e
(`tests/test_turning.py`) must stay green throughout.

## 2. Target architecture: three roles, one seam

```
                      REST API (localhost:5000)
┌───────────────────────────────┐       ◄──────────────────►       ┌─────────────────────┐
│  WORLD SIMULATOR (Python)     │                                   │  GODOT CLIENT       │
│  physics + world rules        │                                   │  rendering + input  │
│  owns all state, hosts API    │                                   │  zero driving logic │
└───────────────────────────────┘                                   └─────────────────────┘
                      ▲
                      │ same REST interface
          ┌───────────┴───────────┐
          │  DRIVERS (any process │
          │  that holds the wheel)│
          │  • e2e suite          │
          │  • AI driver          │
          └───────────────────────┘
```

**World simulator.** Integrates the bicycle model at a fixed 60 Hz timestep;
enforces the world rules (off-road, wrong side, contact stop, physics
validator, lane guard). Receives only primitives. In the end state it does
**no** route/raceline planning (see Phase 2).

**Drivers.** Everyone who holds the wheel — the e2e suite, a human via Godot,
the AI driver — speaks the same small primitive interface:

- *Impulses*: accelerate, brake, steer left/right
- *Intentions*: blinker left/right, U-turn request, destination flag

There is deliberately no "turn left at the next junction" command. The
interface stays primitive so that (a) every scenario remains reproducible
headless ("reproduce first" workflow), (b) the "never do physically
impossible things" rule is enforced in exactly one place — the simulator —
and (c) drivers are interchangeable: a human, a test, and a future phone app
are the same kind of client.

**Godot client.** Pure renderer + input translator: keyboard → `/control`
impulses, blinker key → blinker intention; renders whatever the simulator
reports. It never feeds anything back into physics.

### Collisions stay in the simulator

Contact handling is not mere detection — it is *resolution* (brake at full
A_BRAKE + clamp so the body box never interpenetrates,
`ObstacleManager.apply_contact_stop`, inside the physics substep). It must run
where the state lives, at substep rate, and behave identically headless and
live — e2e asserts contact behavior. Godot's Jolt would be a second source of
truth for the same world; no engine physics is used for the player car. Godot
only *depicts* collision events from state flags (the sim reports what
happened, Godot decides how it looks).

## 3. Geometry: three layers, one contract

Builds on "one curve, all consumers" (`SMOOTH_GEOMETRY_DESIGN.md`): the
angular graph is topology only; the drivable geometry lives in
`SmoothedNetwork` (Catmull-Rom splines + circular junction fillets, resampled
at 0.5 m) and is already shared by physics and rendering today.

| Layer | Content | Consumers |
|---|---|---|
| **1 — Topology** | graph: nodes, edges, connectivity, oneway, lane counts (angular) | route planning ("which edge next?") |
| **2 — Drivable geometry** | `SmoothedNetwork`: splines + fillets, lane widths/offsets, paved area | the simulator's checks, the driver's raceline, Godot's rendering |
| **3 — Render presentation** | colors, textures, camera, oblique/3D view, props, sidewalks | Godot only |

**The rule: layer 2 is the contract.** Layer 3 must project the drivable area
of layer 2 faithfully; it may embellish only *outside* it. If Godot draws a
road that looks paved where the simulator says grass (or vice versa), the
player experiences "impossible" behavior even though the math is clean.

**Oblique/3D views are a camera decision, not a new model.** The world stays
flat (x = east, y = north, 2 px/m); Godot maps it into its XZ plane and an
oblique view is just a camera angle; road width becomes ribbon/extrusion
geometry along the same layer-2 polylines. Physics and model are untouched.
(Elevation/hills would be new data the simulator does not have — flat world
until that changes.)

## 4. The API becomes a product boundary

All existing endpoints stay as they are (the e2e suite keeps working
unchanged). Additions:

1. **`GET /map`** — layer-2 export, served once per map load: merged lines as
   dense polylines (the 0.5 m resample already exists and is cached), each
   with width, lane count, parking-lane width, oneway; junction fillets;
   world bounds; named start points. This is what Godot renders from and — in
   Phase 2 — what the external AI driver plans on.
2. **Freeze/unfreeze command** — ESC-pause exists only in-process today; a
   headless simulator needs an API way to pause.
3. State already carries what Godot needs (car x/y/heading/speed, blinkers,
   hazard, wrong side, off-road, parking state, HUD label, `frame`/`time` for
   interpolation) — no new fields expected; verify at implementation time.

**Transport: REST polling first.** Localhost keep-alive gives ~1–3 ms RTT and
Flask already runs threaded. Free-mode latency = one POST + at most one
physics substep ≈ 2 frames — the same order as today's per-frame key
polling. A WebSocket push channel is added **only if measurement shows a
problem**, not beforehand.

## 5. What moves where

| Concern | Today (in-process pygame) | Target |
|---|---|---|
| Camera / zoom / pan | Python `Camera` | Godot (render concern); `camera_*` state fields retire (`--start` focus becomes a Godot-side initial view, from `/map` start points) |
| Render interpolation | `_render_state` lerp in `main.py` | Godot keeps the last two states and lerps itself (matters on 120/144 Hz displays vs. the 60 Hz sim) |
| Breadcrumb trail | `car.trail`, in-process | Godot accumulates from 60 Hz positions, or the server sends the last N points — decide at M4 |
| Input (keyboard/mouse) | pygame events | Godot → REST |
| Minimap, HUD, flags, obstacle palette, quit button | pygame drawing | Godot UI |
| "Orderly shutdown" convention | window close prints a marker | transfers to the Godot window (e2e must still distinguish deliberate close from crash) |

## 6. Phases

### Phase 1 — frontend swap (behavior-frozen)

Godot renders the existing world in **2D top-down with exact parity to
pygame**; the AI driver stays inside the simulator. Zero behavior change,
e2e stays green. Parity against the pygame output is the acceptance test.

- **M1** — DONE: `GET /map` + Godot renders the static map (roads,
  markings, junction dots, colors per current config)
- **M2** — DONE: car appears and follows (60 Hz state polling,
  interpolation with 100 ms render delay, camera mirroring the sim's
  camera, minimap)
- **M3** — DONE (visual parity): car sprite + blinkers + wheel
  breadcrumbs, test flags, HUD label, smooth rendering pipeline (double
  HTTPRequest pipeline, 1 s-window sim-rate estimate, extrapolation).
  NOTE: the original M3 scope "free-mode driving from the Godot window"
  was deferred to AFTER M5 by decision.
- **M4** — PARTIAL: minimap/HUD/flags/trail done; remaining: obstacles +
  palette, freeze overlay
- **M5** — DONE (user-reordered ahead of free driving): pygame is out of
  the package. `renderer.py`, `obstacle_ui.py`, `road_surface.py` deleted;
  the sim runs headless at a self-paced 60 Hz (hybrid sleep+spin pacing -
  plain `time.sleep` overshoots ~50% on this platform); flag/HUD state
  moved to a `TestFlags` object in `main.py`; `Camera` kept as pure state
  (Godot mirrors it via `/state`); new `POST /freeze` replaces the ESC key;
  `GET /screenshot` removed; `docs/TESTING.md` workflow updated.
  **Next: M3-free** — free-mode driving from the Godot window via REST
  (also measures real input latency for the Phase-2 decision)

### Phase 2 — planning outside (goal state)

Route + raceline + speed profile move out of the simulator into an external
**AI-driver process**; the world simulator then receives only impulses.

Known cost, stated up front: the control loop gains ~1–2 frames of latency,
and the cornering tuning sits at the 0.35 m / ~35 ms edge
(`ROAD_EDGE_TOLERANCE_M`). Expect a re-tuning iteration (preview distance,
corner speeds) under e2e supervision. The `/map` contract from Phase 1 is
designed so the external driver gets layer 2 with no further API work.

## 7. Repos & migration

This is a **permanent split, not a feature branch**: the Godot frontend is
not going away — pygame is. Framing it as "a branch in `car/`" would be
wrong, because a branch implies merging back later.

- **`car/`** stays the simulator repo. The Python-side changes are small and
  additive (new endpoints, freeze command) → short-lived branch cut from the
  current line of development, validated by e2e, merged quickly. Pygame is
  deleted at M5 on the mainline — it does not live on in a branch forever.
- **`driving-game/`** becomes the frontend repo (`git init`; currently
  unversioned). All Godot work happens there, as a proper standalone Godot
  project at the repo root.
- If the two-repo workflow proves annoying later, consolidating into one
  monorepo stays open — the documented API contract is the seam either way.

## 8. Open questions / risks

- **Phase-2 re-tuning scope** — to be measured, not guessed: M3 gives real
  free-mode latency numbers with plain REST polling; that measurement decides
  whether Phase 2 needs WebSocket push or just controller re-tuning.
- **Trail source** — client-side accumulation vs. server-sent points (M4).
- **Process lifecycle** — for now a dev script starts simulator + Godot;
  later, optionally, Godot spawns the simulator as a child process (single
  entry point, clean teardown).
- **Oblique/3D view** — pure camera/extrusion work after parity; deferred by
  design, not by lack of plan.
