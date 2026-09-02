# Multi-Car Plan (parallel test runs)

Goal: several cars on the map at once, each with its own driver state,
controls, test flags and HUD label — so e2e scenarios can run in parallel.
The Godot frontend is **already multi-capable** (`_carNodes` per uid, TAB
cycles between cars, F toggles follow, and the `/state` parser prefers a
`cars: [...]` array when present). This plan is mostly sim-side work plus
small Godot additions.

## Design decisions (binding for this phase)

1. **Teleport semantics** — `POST /teleport` keeps its legacy behavior by
   default: it **replaces** the current car(s) (exactly what the e2e suite
   and cockpit controller do today). A new optional parameter
   `"add": true` **adds** a fresh car alongside the existing ones. This
   keeps every existing consumer working unchanged; parallel runners opt
   in explicitly.
2. **Primary ("followed") car** — the most recently spawned car. The sim
   camera follows it, and ALL legacy top-level `/state` fields describe
   it (so `test_turning.py`, `_wait_for_new_car`, and `tools/controller.py`
   keep working untouched). Removing the followed car falls back to the
   newest remaining one; no cars → the existing `has_car: false` branch.
3. **Per-car addressing** — `/control`, `/flags`, `/label`, `/toggle` and
   the hazard command accept an optional `"uid"`. Without `uid` they apply
   to the primary car (legacy behavior). With `uid` they apply to that car.
4. **Cleanup** — new `POST /cars` endpoint: `{"action": "clear"}` removes
   all cars, `{"action": "remove", "uid": N}` removes one. Parallel runners
   clear between rounds; also handy for dev.
5. **No car–car collisions in this phase** — moving cars pass through each
   other (today's behavior). SAT stop-on-contact exists only for static
   obstacles. Collisions are a separate, later decision.
6. **`/state` shape** — legacy top-level fields stay (primary car) AND a
   new `cars` array is always exported (empty when no cars). Godot already
   prefers the array; it switches over automatically.

## Car colors (user decision)

Cars get distinct colors from the old pygame palette: **red #B41E1E**
(player color, car #1), then **blue (65,105,220)**, **yellow (235,195,45)**,
**white (238,238,238)** — the three obstacle-palette colors from the old
pygame. Assignment is deterministic per uid: `index = (uid - 1) % 4`.
The sim exports a `color` name per car (top-level + in `cars`); Godot
preloads four sprite variants generated ONCE from the red base sprite with
the exact old pygame tint formula (`new_c = clip(lum/96 × target_c)` —
see deleted `tinted_car_sprite`), committed as PNGs in `assets/`.

## Car object contract (per entry in `cars`)

```json
{
  "car_uid": 3,
  "x": ..., "y": ..., "heading": ...,          // rear axle, world px (as today)
  "speed_kmh": ..., "level": ...,              // what Godot reads today
  "segment": ..., "progress": ...,             // for per-car readouts
  "on_road": ..., "wrong_side": ...,
  "blinker_left": ..., "blinker_right": ...,   // per-car blinker lights
  "hazard": ...,
  "color": "yellow",                            // red|blue|yellow|white
  "flags": {"green": [x, y, h] | null, "red": [x, y, h] | null},
  "hud_label": "7/21" | null
}
```

---

## Step 1 — `src/rest_api.py`: per-uid control + new endpoint

- [x] `control_input` becomes `Dict[Optional[int], Dict[str, bool]]`
      (key = uid; key `None` = "unaddressed" → primary car only)
- [x] `POST /control` accepts optional `"uid"` (popped before the loop over
      control keys); without it → key `None` (legacy)
- [x] `get_control_for(uid, is_primary)` → explicit uid's dict if present,
      else the `None` dict when `is_primary`, else all-false
- [x] `clear_control(key, uid, is_primary)` with the same resolution
      (one-shot blinker/uturn consumption must hit the SAME bucket)
- [x] `POST /toggle`, hazard in `/control`, `POST /flags`, `POST /label`
      pass an optional `"uid"` through into `commands` unchanged
- [x] New `POST /cars`: `{"action": "clear"}` | `{"action": "remove",
      "uid": N}` → `commands['cars'] = data`
- [x] `/teleport` handler unchanged (the `add` flag rides in the params;
      the main loop interprets it)

## Step 2 — `src/main.py`: cars dict + loop rework

- [x] `cars: Dict[int, Car] = {}` replaces the single `car`; `_follow_uid:
      int | None` tracks the primary car
- [x] Teleport command: `"add": true` → add; default → clear then add.
      Both set `_follow_uid` to the new car and snap the camera (as today)
- [x] `commands['cars']`: implement clear / remove (removing the followed
      car re-points follow at the newest remaining, or None)
- [x] Per-car flags: `car_flags: Dict[int, TestFlags]`; `/flags` + `/label`
      commands with uid → that car's `TestFlags`, without → primary's
      (create on demand; drop entries when a car is removed)
- [x] Physics substep loop iterates **all** cars (shared accumulator —
      every car steps dt_fixed in the same substep, deterministic):
      per-car control merge (driver + API bucket), `car.update`,
      `obstacle_mgr.apply_contact_stop`, `validator.check`,
      `lane_guard.check` (wrong side), FREE-mode off-road stop
- [x] Per-car frame outputs (`on_wrong_side`, blinkers, …) collected into a
      dict keyed by uid for the state export
- [x] Per-car render interpolation: `_prev_render` becomes a per-uid dict
      (teleport-snap detection stays per car)
- [x] Red-flag resolution loop over all cars with a pending flag (route
      check against THAT car's `bicycle_nav`, destination set on THAT nav)
- [x] Camera follows the followed car's interpolated position (unchanged
      mechanics, new source)
- [x] `/state` export: top-level fields unchanged (primary car; no-car
      branch unchanged) + always-present `cars` array with the contract
      above (per-car flags/label from its `TestFlags`)

## Step 3 — Godot (`driving-game/MapRenderer.cs`): per-car visuals

- [x] Generate 3 sprite variants (blue/yellow/white) from the red base
      with the old pygame tint formula; commit PNGs to `assets/`
- [x] Per-car parse: read `blinker_left/right`, `hazard`, `color`,
      `flags`, `hud_label` from each entry of the `cars` array (store in a
      small per-uid struct next to `_carNodes`)
- [x] Blinker lights: use the per-uid values instead of the single-car
      top-level read (the current `!TryGetProperty("cars")` guard goes away)
- [x] Flag pennants: draw one green/red pair PER CAR from its entry; fall
      back to the legacy root-level `flags` only when no `cars` array is
      present (old sim compatibility)
- [x] HUD: big top-right label = followed car's `hud_label`; small per-car
      label above each car that has one (so parallel tests are identifiable
      at a glance)
- [x] Minimap: one dot per car (followed = red as today, others white/yellow)

## Step 4 — Verification

- [x] Unit tests green (`pytest tests/`, known-flaky `test_api.py::test_basic_driving` deselected)
- [x] Smoke test (`--smoke 300`)
- [x] **Full e2e suite 21/21** — proves backward compatibility (replace
      semantics + top-level fields untouched)
- [x] Manual multi-car in the Godot window: teleport 3 cars with
      `add: true`, distinct per-car flags + labels + blinkers; TAB cycles
      the camera; all three visible with own pennants/labels/trails
- [x] Pacing still exactly 60 fps with 3 cars (frame counter over wall time)
- [x] `POST /cars` clear/remove verified via curl

## Step 5 — Docs, KB, commit

- [x] `docs/REST_API.md`: `/control`+`/flags`+`/label`+`/toggle` optional
      `uid`; new `POST /cars`; `/teleport` `add` flag; `/state` `cars` array
- [x] `docs/TESTING.md`: short "parallel runs" note (add:true + /cars clear
      pattern) — the sequential suite itself is unchanged
- [x] Knowledge base entry: multi-car architecture (primary car = followed,
      top-level /state = primary, cars array preferred by Godot, add flag)
- [x] Commit both repos locally (car/ `0e89887`, driving-game/ `dcba360`);
      push after user says "tested"

## Step 6 — Multi-car stress scenario (`fig8_stress`)

- [x] `POST /teleport {"segment": N, "progress": p}` - spawn on an
      arbitrary segment (chord placement), used by the stress test
- [x] Scenario in tests/test_turning.py: phases 2/6/10/14/18 cars on the
      figure-8, 30 s each; per-phase jitter sum stored in
      turning_results.json (`fig8_stress|<N>cars` → `jitters`,
      `worst_jump_m`)
- [x] **Fixed the command-ordering bug this exposed**: the per-key command
      queue applied all teleports before any same-frame `/cars` clear, so a
      "clear then spawn" burst lost its order. Now ONE global FIFO of
      `(key, payload)` (rest_api.py) applied in strict arrival order
      (main.py). e2e suite 22/22 afterwards.
- [x] Result: 0 jitters up to 18 cars, sim holds exactly 60 fps

## Explicitly OUT of scope (later decisions)

- Car–car collision physics (SAT exists for static obstacles; reusing it
  for moving pairs is a separate task with its own design questions:
  full stop vs. yield, which car yields, validator interaction)
- A parallel e2e runner itself (threads in `test_turning.py` or a new
  script) — the sim will be ready for it; building the runner is a follow-up
- Per-car cockpit views in `tools/controller.py`
