# Turn-Planning Rework — Plan

> **TL;DR (2026-08-16):** The rail model is **retired**; one kinematic
> **bicycle model** is the physics for everything (§8) — AI mass, player-
> assisted, player-free. Roadmap: smoothed geometry (§10) → "Miss Daisy"
> offline reference-line authoring (§9) → pygame rendering + paint-bucket
> trails (§11) → ~500-car traffic simulation of Kleinmachnow (§12).
> **Next task: fix the `sliver_approach` stall — §13.6.**
> **New here? Read §13 (Onboarding) first, then §8.** Hard rule:
> `AGENTS.md` — never do physically impossible things.

## 0. Status (updated 2026-08-16)

**Decision (2026-08-16): the bicycle model becomes THE physics; the rail
model is retired.** End goal: a traffic simulation of one district
(Kleinmachnow) with several hundred (~500) concurrent vehicles running
start→end trips — to find bottlenecks and jams and to evaluate structural
changes (e.g. road closures) — PLUS fun driving: a player-assisted mode
(gas/brake/turn signal, the car holds the line) and a free mode (direct
steering, wrong-way driving allowed). One bicycle model serves all three
operating modes; the rail model fits none of them (a "train on the graph"
cannot represent real gaps/jams, cannot be driven free, and its seam
teleports don't scale). New plan: §8–§13. §3 (rail Phases 1–3), §5 and the
rail items in §7 are **obsolete** (kept for the record).

**Phase 1 (items 1–3) is implemented** in the working tree (NOT committed):
no-plan centerline blend, slide-past + dead-end crash, rate-limited heading
transition, overshoot carry-over, hand-off lane-offset records.
`scripts/repro_crash.py` confirms: the original 815→1008 crash is **fixed**
(car slides past, blinker stays on).

**New crash found & root-caused (818→746, 0.9 m teleport at arc-end
hand-off):** `_lane_offset_factor` is a fraction of the *current segment's*
width/4, so its absolute meaning changes when segment width changes. The
arc is built with an absolute `lane_offset_m` (1.75 m = the 7 m FROM road's
lane); the 3.5 m exit road's quarter width is 0.875 m, so the same factor
renders 0.87 m instead of 1.75 m → lateral teleport. (Masked before because
the car died at 815 first.)

Done / open for this bug:
- [x] `_advance_active_turn`: derive the factor from the arc's absolute
      `lane_offset_m / (to_seg.width/4)` at the hand-off (factor may be up
      to 2.0 = road edge; recovery blend eases it back to 1.0). Verified:
      factor = 2.0 at hand-off.
- [ ] no-plan branch: `approach_to_centerline` is capped at 1.0 → snaps
      2.0→1.0 on the next frame. Anchor the ramp to the hand-off state
      whenever the standard "last 20 m" ramp does NOT pass through it
      (hand-off factor F, hand-off-to-junction distance R): use the standard
      ramp only if `F <= 1.0 and R >= 20*F`, else
      `approach = F * remaining_to_junction / R` (passes through (hand-off,
      F) and (junction, 0); `min(recovery, approach)` of two curves sharing
      the hand-off point is continuous there by construction).
- [ ] plan branch: drop the `abs(recovery_start - 1.0) > 1e-6` guard so the
      ease ALWAYS starts from the actual hand-off factor (it currently skips
      the ease exactly when `planned_factor(hand-off) != 1.0` — same bug
      class).
- [ ] then full §4 verification (repro, test suite, long runs).

Working tree: `src/car.py` modified, `scripts/repro_crash.py` +
`scripts/debug_handoff.py` untracked. Nothing committed. **The repro still
crashes** (frame 261) until the two open fixes above are in.

**Direction change (brainstorm 2026-08-16, see §6):** the rail model is
identified as the root cause of this whole bug class. Agreed next step is a
one-car bicycle-model prototype — Phases 2–3 below may be superseded.

---

## 1. The problem (verified against the OSM data)

Crash loop: the game freezes and dies every time the AI car reaches the
junction **815 → 1008** (Kleinmachnow OSM map).

Facts (verified):

| | value |
|---|---|
| seg 815 (approach) | **4.2 m** long, 7 m wide |
| the node | a **4-way junction**: 816 continues (1.9°), 1007 right (91.9°), 1008 left (−87.6°) |
| seg 1008 (the signaled left turn) | 19.8 m long, 7 m wide |
| tangent distance needed (slowest valid plan, 18 km/h) | 4.8 m |
| road left when the blinker was set | 3.7 m |

The blinker (left) was set ~3 m before the junction — too late for any
physically possible turn into 1008. Per the agreed blinker semantics
(see 2.5), the correct behavior is to **slide past the junction and
continue on 816**, blinker still on, taking the next reachable left
turn. Instead, the code force-entered 1008 and teleported.

Chain of failure:

1. The turn is only planned while the car is **already on 815** (the
   segment directly before the junction).
2. No arc radius fits in 4.2 m of approach road → planner gives up at
   every speed down to the mechanical minimum → `plan = None`.
3. The "no plan" branch **does not brake** (only genuine dead ends
   brake) → the car barrels into the corner at 57 km/h.
4. At the node, `_handle_segment_end` instant-swaps to seg 1008. The
   right-lane offset points in a different direction on the new segment
   → **2.42 m lateral teleport** (measured: matches the crash's 2.4 m).
5. The heading also snaps 87.6° in one frame.
6. The physics validator (correctly) flags both → `RuntimeError` →
   game process dies.

Root cause: **turn planning is anchored to segment boundaries**, but a
real turn is a continuous maneuver on the road *surface*. The tangent
point (where the arc must begin) can lie on the segment *before* the
junction — and the car only starts thinking about the turn once it is
too late.

## 2. Principles (agreed)

1. **Never do physically impossible things** (standing rule,
   `AGENTS.md`). No teleports, no instant heading changes, no radius
   below the mechanical minimum. Fix the logic, never weaken the
   validator.
2. **The road is a continuous surface**, not a list of segments.
   Segments are an implementation detail; the car thinks in
   "distance to the point where I must act".
3. **The blinker is the decision point.** The moment the turn is
   decided, the car must be far enough from the tangent point to still
   brake and enter the arc safely. All braking/steering logic after
   that is a function of **distance to the tangent point** — not of
   which segment the car happens to be on.
4. **Two tiers of road surface:**
   - *physically drivable*: the whole street (both lanes, shoulder);
   - *permitted*: our own lane.
   The opposing lane is only used when **clear of oncoming traffic**,
   and only as a last resort (sometimes unavoidable on small roads).
5. **Blinker semantics — "next reachable decision point".** Setting the
   blinker means: *I want to turn this way at the next decision point
   where it is still physically reachable* (enough distance to brake
to a speed whose arc radius and tangent distance both fit). A blinker
   set 3 m before a junction does **not** mean "turn at this junction"
   — it means "at the next junction where I can still make it". If the
   signaled turn is unreachable and the car cannot brake in time, it
   **slides past**: continues on the straight continuation if the road
   goes on, or **crashes** into the obstacle at the end of a T-junction.
   Crashing is physically possible and allowed; a teleport, a
   spot-turn, or a 5 m radius at 60 km/h are not.
6. **Driver personality comes later** (cautious: stays in-lane,
   opposing lane only in extremis; sporty: cuts corners for speed).
   Both as their own classes. → the lane-usage strategy must be
   **pluggable** from the start.

## 3. Changes

> **OBSOLETE (2026-08-16).** The rail model is retired (§0, §8). These
> phases are kept for the record only — do not implement. The bicycle model
> replaces them: Miss Daisy reference lines (§9) + one bicycle physics for
> all three operating modes (§8).

### Phase 1 — Stop the crash (safety net, small)

Even with Phase 2, pathological geometry (hairpins, sliver segments)
can still yield "no arc fits at all". The fallback must then still be
physically legal:

1. **`Car._update_position_rails`, "no plan" branch** — blend
   `_lane_offset_factor` → 0 (centerline) over the last ~20 m before
   the junction (same blend the normal branch already uses). On the
   centerline both segments share the exact node point → the
   segment-end hand-off has **no lateral jump, ever**.
2. **Slide past, don't force the turn.** When the signaled next
   segment exists but no arc fits (unreachable turn), the car does
   **not** enter it: it continues on the straight continuation (the
   smallest-angle exit, e.g. 816 in the crash case) with the blinker
   still on, looking for the next reachable decision point. Only if
   there is no continuation at all (T-junction dead end) does the car
   **crash**: a physical event — rapid deceleration to 0 over a short
   distance against the obstacle, no teleport, no instant stop from
   speed. (Currently only `_pending_junction_is_dead_end` brakes, and
   even that just stops the car in the air.)
3. **Smooth the heading across the swap** — reuse the existing
   `_heading_transition` mechanism (0.3 s smoothstep) for the
   segment-end hand-off when no arc was executed (matters mostly for
   sharp continuations; the 815→816 case is only 1.9°).

### Phase 2 — Look-ahead planning on the continuous surface (the real fix)

4. **Plan from the segment before the junction.**
   `Car._update_position_rails` already plans for the junction at the
   end of the *current* segment. Add: when the planned tangent point
   for that junction would lie **before the start of the current
   segment** (`from_tangent_offset_m > seg.length`), the plan must
   have been made one segment earlier. Concretely: while on segment N,
   also evaluate the junction at the end of N and create the plan
   there, so the tangent point (which may sit on N) is known and the
   car can already be braking for it.
5. **Braking keyed to distance-to-tangent, not to segment position.**
   The existing `distance_to_tangent_m` computation already does this
   within one segment; with (4) it becomes valid across the
   segment boundary, because the plan exists before the car reaches
   the tangent segment. No new braking formula needed — the current
   `braking_distance + 5 m margin` check stays, it just fires earlier.
6. **Blinker targeting = next reachable decision point.** When the
   blinker is set, the AI evaluates the upcoming junction: if the
   signaled turn is reachable (braking distance + tangent distance
   both fit), the plan targets *this* junction and braking starts
   immediately; if not, the intent is deferred to the next junction
   in the signaled direction (the car keeps driving, re-evaluating).
   This is the direct implementation of principle 2.5.
7. **Late decision (tangent point already passed)** → Phase 1
   behavior: slide past (or crash at a dead end), never teleport.

### Phase 3 — Opposing-lane policy + pluggable driver style

8. **`TurnStyle` (new small class, e.g. in `turning_system.py` or its
   own module)** — encapsulates *how* a driver uses the road surface:
   - `candidate_lane_offsets()`: ordered list of lane-offset fractions
     to try (today: own lane `1.0, 0.5, 0.0`, then opposing `-0.5`);
   - `target_speed_for_turn(angle, cruise)`: speed preference
     (today: the severity table in
     `TurningSystem.decide_target_speed_for_turn`);
   - `may_use_opposing_lane(network, from_seg, to_seg, node)`:
     clearance check. With no traffic modeled yet this returns True,
     but the **call site exists** so traffic can plug in later without
     touching the planner.
9. **`CautiousDriverStyle`** (default, = today's behavior): own lane
   at every speed first, opposing lane only after all own-lane speeds
   down to the mechanical minimum failed.
10. **`SportyDriverStyle`** (later, stub now if at all): prefers
    speed, cuts corners (accepts opposing-lane usage earlier), higher
    target speeds within the physics limits.
11. **`Car._get_or_create_planned_turn`** iterates
    `style.candidate_lane_offsets()` / `style.target_speed_for_turn()`
    instead of hard-coding the lists — the planner itself is unchanged.

### Out of scope (for now)

- Traffic simulation / oncoming-vehicle detection (only the hook).
- AI turn *direction* decisions (left/right at real junctions) — the
  AIDriver currently follows "straight" unless a key is pressed; that
  stays as is.
- FREE-mode (keyboard) physics.

## 4. Verification

1. **Reproduce first**: headless run that drives the car onto seg 815
   heading toward 1008 (teleport-to-segment + RAILS), confirm the
   current crash.
2. **Unit-ish checks**:
   - no-plan fallback: car crosses a no-arc junction → position
     continuous (< 0.1 m/frame beyond `speed·dt`), heading change
     ≤ 30°/frame, validator stays enabled and silent.
   - look-ahead: plan for a junction whose tangent point lies on the
     previous segment exists *while still on that previous segment*.
   - opposing-lane policy: `CautiousDriverStyle` never returns a
     negative offset before all own-lane candidates are exhausted.
3. **Full game run** with the validator enabled: drive the OSM map for
   several minutes (API + random teleports), zero `RuntimeError`s.
4. Existing test suite stays green (the 3 pre-existing failures in
   `test_api.py` / `test_random_road_point` are unrelated and remain).

## 5. Order of work

> **OBSOLETE (2026-08-16)** — superseded by the work order in §8.5.

1. Phase 1 (1–3) — smallest diff, kills the crash loop immediately. **Done (uncommitted).**
2. Phase 2 (4–7) — the real fix; the no-plan fallback becomes rare.
3. Phase 3 (8–11) — policy extraction; behavior-preserving refactor,
   sporty style as a stub.

## 6. Direction change: physical car model (brainstorm 2026-08-16)

**Diagnosis:** nearly every crash/teleport bug is a symptom of the rail
model — the car is a train on the OSM graph, and physics only exists at the
seams (junction hand-offs). OSM edges/nodes are a *logical* network, not a
physical surface.

**Agreed direction:** one physics model for AI AND player (FREE mode =
same car, keyboard instead of AI control):
- **Kinematic bicycle model**: state (x, y, heading, v, steering angle);
  steering angle × speed → turning radius (same angle, more speed = wider
  arc). Lateral-accel limit → understeer is *emergent*, not planned.
  No drift/oversteer (force model = overkill at scale).
- **Two-tier driver:**
  1. *Intent* (event-driven, low-frequency): OSM graph decisions — which
     junction, turn direction, blinker, target speed profile, lane choice.
  2. *Control* (per frame, a few flops): **pure pursuit** — look at a
     point ahead on the reference line (lookahead ∝ speed), steer toward it.
     Reacts any time, steers more or less strongly, pulls the car back when
     it deviates. Braking for curvature from the precomputed speed profile.
- **Reference lines are static geometry**: smooth centerline (+ lane
  offset) precomputed once at map load, shared by all cars. Per car: route
  + speed profile only.
- **Deviations are allowed** (like in the real world); avoidance of other
  cars = lateral shift of the pursuit target / steer away, with priority
  brake > dodge > hold (traffic = later, needs a spatial grid for
  neighbor lookup — 500 cars stay cheap because control is open-loop-ish
  and perception is local).
- **Crashes stay allowed, teleports don't** — with an integrated model this
  becomes emergent; the validator becomes a safety net instead of a wall.

**Prototype scope (agreed, first step):** ONE car, bicycle model, no
traffic — it must **stay on the road** (`network.is_on_road`, the paved
polygon). Standalone script (planned: `scripts/proto_bicycle.py`), reusing
`src.road_network` geometry — does not touch `src/` yet.

**Next steps:**
1. Build & run the prototype; success = on-road through several junctions
   incl. tight corners (brake + steer from the model, no rail logic).
2. Decide: adopt the new physics for AI + player (this plan's Phases 2–3
   become obsolete; Phase 1's hand-off/teleport fixes get retired with the
   rail model) — or keep the rail model and finish Phase 1's two open
   fixes from §0.
3. (Later) traffic + avoidance, driver personalities (§3 Phase 3 ideas map
   onto the intent tier).

---

## 7. TODO (updated 2026-08-16, after bicycle-model integration)

### Done — bicycle model (uncommitted, working tree)

- [x] **Prototype** `scripts/proto_bicycle.py`: standalone, reuses
      `src.road_network`, on-road through junctions + tight corners.
- [x] **Integration** into the game: `src/bicycle_nav.py` (BicycleNav
      class), `BicycleDriver` in `src/driver.py`, `--bicycle` flag in
      `src/main.py`.
- [x] **Steering bug** (local-frame rotation signs) fixed.
- [x] **Roundabout direction** fixed: ring is now COUNTER-CLOCKWISE
      (correct for Germany / right-hand traffic; island stays on the left).
- [x] **Right-hand lane offset** implemented (`_offset_polyline_right`,
      `LANE_OFFSET_M = 1.75` for 7 m two-way roads).
- [x] **Car-width on-road check**: `RoadNetwork.is_car_on_road()` checks
      four corners of a 4.5 m × 1.8 m car against the paved polygon.
- [x] **Roundabout ring densified**: 8 nodes → 64 nodes (chord curvature
      was 0 on straight chords, so the speed profile didn't slow for the
      ring).
- [x] **Look-ahead curvature** in the speed profile (max |κ| over a 60 m
      window, not just local chord curvature).
- [x] **Lateral-accel cap** lowered: `A_LAT_MAX = 2.0 m/s²` (from 3.5).
- [x] **Dead-end stop**: terminal speed=0 when the route ends at a
      degree-≤1 node (prevents driving off the end of short segments).
- [x] **Route stability**: `_maybe_rebuild` no longer rebuilds on every
      node crossing (only when: no line, turn changed, or line
      exhausted). Added `_route_seg_set`.
- [x] **Roundabout exit**: after the first junction on a one-way ring,
      take the first non-oneway exit (`_next_after_first`).
- [x] **Corner radius** = 6.0 m (`config.ROAD_CORNER_RADIUS_M`), matching
      the renderer's paved fillet.
- [x] **Test map** rewritten in OSM coordinates (X east, Y north,
      heading 0 = north, forward = (sin h, cos h)); sliver junction
      (segment-815 layout) added; autobahn removed.
- [x] **Tests** rewritten to match the new map (expected end segments
      re-verified; left/right swapped due to the OSM Y-north convention).
- [x] **Lane offset for narrow roads**: offset is now capped by the
      narrowest segment in the route (`min_width/2 - 0.9 - 0.1`), so a
      3.5 m one-way road doesn't push the car's right wheels off the
      road.
- [x] **Lane offset for left turns**: halved for left turns (the right
      side is on the outside of the turn; a full offset makes the car
      swing wide).
- [x] **Realistic car speed**: 0–100 km/h in ~10 s (`CAR_ACCELERATION =
      2.8 m/s²`), top speed 200 km/h (`CAR_SPEED = 55.6 m/s`); the car
      accelerates as far as it can on straights and brakes only when the
      speed profile requires it.
- [x] **Headless performance**: ~61× faster than real-time (6000 frames
      in 1.6 s), so ~1 hour of driving simulates in ~59 s.

### Open — bicycle model (blocking)

- [ ] **Fix `sliver_approach`**: the car is stuck at s=0 (speed 0.0,
      `accel_scale` = 0.0 because the steering angle is large at the
      start). The reference line curves to the west at the sliver
      junction, so the pure-pursuit target is to the left of the car's
      heading, and the accel-scale gate (throttle off while steering)
      prevents the car from ever moving. Need to either (a) reduce the
      steering angle at the start (e.g. a smaller base lookahead, or a
      gentler corner radius at the sliver junction), or (b) allow some
      acceleration even while steering (lower the accel-scale gate's
      threshold).
- [ ] **Fix the 12 timed-out tests** in the visible-window REST-API test
      (`tests/test_turning.py`). Most time out because the car never
      reaches the expected end segment within the 15 s monitor window
      (related to the `sliver_approach` stall, and possibly to the car
      starting at the same segment as the expected end segment).
- [ ] **Test the bicycle model on the OSM map** (so far only tested on
      the synthetic `basic` map). The OSM map has real-world geometry
      (tight corners, sliver segments, one-way rings) that the synthetic
      map only approximates.

### Open — decision

- [x] **Adopt or retire**: DECIDED (2026-08-16) — adopt the bicycle model
      for everything; the rail model is retired. See §0 and §8.
- [ ] **Commit** the bicycle-model work (currently uncommitted in the
      working tree: `src/bicycle_nav.py` new, `src/car.py`,
      `src/config.py`, `src/driver.py`, `src/main.py`,
      `src/road_network.py`, `src/test_maps.py`, `tests/test_turning.py`
      modified, `scripts/` untracked).

### Obsolete — rail model, Phase 1 (retired with the rail model, 2026-08-16)

- [ ] ~~no-plan branch: `approach_to_centerline` capped at 1.0 → anchor
      the ramp to the hand-off state.~~
- [ ] ~~plan branch: drop the `abs(recovery_start - 1.0) > 1e-6` guard.~~
- [ ] ~~Full §4 verification of the rail model.~~

### Obsolete — Phase 2 (rail look-ahead planning, retired 2026-08-16)

- [ ] ~~Plan from the segment before the junction (item 4).~~
- [ ] ~~Braking keyed to distance-to-tangent (item 5).~~
- [ ] ~~Blinker targeting = next reachable decision point (item 6).~~
- [ ] ~~Late decision → slide past or crash (item 7).~~

### Obsolete — Phase 3 (rail driver styles, retired 2026-08-16)

- [ ] ~~`TurnStyle` / `CautiousDriverStyle` / `SportyDriverStyle` as
      planner policies (items 8–11).~~ Driver personality re-enters as a
      *speed-profile* parameter on shared reference lines — see §9.5.

### Open — later (traffic, personalities, FREE mode)

> Now mostly covered by §8 (operating modes), §9.5 (personalities as
> profile parameters) and §12 (traffic simulation) — items below remain
> open only insofar as they go beyond those sections.

- [ ] Traffic simulation / oncoming-vehicle detection (only the hook for
      now).
- [ ] AI turn *direction* decisions (left/right at real junctions).
- [ ] Driver personalities (cautious vs sporty) as pluggable classes.
- [ ] FREE-mode (keyboard) physics — same car, keyboard instead of AI
      control.
- [ ] Crash handling as a physical event (rapid deceleration against the
      obstacle, no teleport, no instant stop from speed).

---

## 8. Decision: one bicycle model, three operating modes (2026-08-16)

**The rail model is retired.** End goal (agreed):

1. **Traffic simulation, one district (Kleinmachnow):** several hundred
   (~500) vehicles concurrently, each running a start→end trip. Purpose:
   find bottlenecks and jams, measure where, and evaluate structural
   changes (e.g. "should road X be closed?").
2. **Fun driving, two player modes:**
   - *assisted* (the old "rail-like" wish): the player gives gas/brake +
     turn signal (intent); the car holds the line and drives the curves;
   - *free*: the player steers directly left/right however they want —
     wrong-way driving (Geisterfahrer) explicitly allowed.

**Why the rail model fits none of these:**

- 500-car simulation: a "train on the graph" cannot represent real gaps
  and headways, so jams never emerge; seam teleports don't scale.
- assisted mode: "the car drives the curves, I give gas/blinkers" is a
  car *following a line with a human as intent driver* — not a train.
- free mode: a train on the graph *cannot* be a wrong-way driver by
  definition.

**The three modes are one physics with different steering sources:**

```
Bicycle model (x, y, heading, v, steering angle) — ONE physics for all
    |
    +-- A. AI mass:      reference line (Miss Daisy, §9) + speed profile
    |                   + local reaction (brake for car ahead)
    |                   -> pure pursuit on the line. Cheap, safe.
    |
    +-- B. Player assisted: SAME line + profile as A, but the human
    |                   supplies gas/brake and the turn signal (intent =
    |                   which line next). Car holds the line via pursuit.
    |
    +-- C. Player free:   NO reference line. Human supplies steering
                          angle + gas/brake directly. The car does what
                          you tell it — including wrong-way.
```

A and B share ALL infrastructure (reference lines, profiles, pursuit,
on-road check, trails); B is A with a human at the throttle. C reuses
the same physics with the reference line switched off.

### 8.1 Consequences

- The rail model's open items (Phase 1 fixes, Phases 2–3) are **obsolete**
  and retired with it.
- The two open bicycle bugs (`sliver_approach` stall, 12 timed-out tests)
  stay **priority**: Miss Daisy IS a bicycle driver — if she stalls on the
  sliver junction, she cannot author that line.
- "Test the bicycle model on the OSM map" **becomes the authoring run
  itself** (§9.6).
- The validator stays as a **safety net** (never weakened), not a wall:
  with precomputed, proven-drivable lines, violations become rare events
  worth logging, not the norm.

### 8.2 Work order (replaces §5)

1. Fix `sliver_approach` stall + the 12 timed-out tests (bicycle model
   must be trustworthy before it authors anything).
2. Smoothed geometry pipeline (§10): centripetal Catmull-Rom through the
   original OSM nodes, κ(s), shared by renderer / driving / on-road check.
3. Miss Daisy authoring on the OSM map (§9): every directed way + every
   junction turn → precomputed lines + speed profiles; verify on-road.
4. Player modes: assisted (B) and free (C) on the same physics.
5. Paint-bucket trail overlay (§11) — global + per-vehicle toggle.
6. Traffic simulation (§12): exit points, trips, steady state, metrics.
7. Commit (see §7 "Open — decision").

---

## 9. Miss Daisy: offline reference-path authoring

**Idea (agreed):** use the bicycle model as an *offline authoring tool*,
not a runtime planner. Drive the whole network once with an ultra-
cautious virtual driver — **Miss Daisy**: always in her lane, never in
oncoming traffic, never cuts corners, brakes early. What she produces is
a set of **precomputed, guaranteed-drivable reference paths**. The mass AI
drivers stop planning at runtime; they just *follow* precomputed paths.

The hard, expensive, error-prone part (finding a physically feasible line
through every junction) is done **once, offline, with unlimited compute
and no real-time pressure**. At runtime only the easy part remains
(follow a line already known to work) — which is why the rail-model
crash class (teleports, impossible turns) disappears: paths are
*feasible by construction*.

### 9.1 Unit of precomputation: per maneuver

- ❌ per full *route* — combinatorially infinite.
- ❌ per *edge* only — misses the turns, where the dangerous geometry is.
- ✅ **per maneuver**: each **directed way** (straight-through piece)
  PLUS each **in-way → out-way turn at every node**.
  - degree-2 node (two ways meet): exactly **one** forced continuation
    piece — no decision, but the connection geometry is still computed
    (a short blend if the ways meet at a kink; nothing if already
    tangent).
  - degree ≥ 3 node: **one piece per (in, out) pair** — the real choices.

Any route is then a stitch of these pieces. Degree-2 seams are numerous
but trivial (near-zero blends); degree-3+ turns are few but interesting
— that is where Miss Daisy's caution actually matters.

### 9.2 What a precomputed path stores

- The polyline **she actually drove** (lane offset included, corner not
  cut) — the line is the geometry, proven by the run.
- A **safe speed profile** along it (her cautious profile = the
  guaranteed-safe baseline; see §9.5 for per-driver rescaling).
- Curvature κ(s), length, and the connect points (where it meets the
  next piece).

### 9.3 The speed profile from curvature

The spline gives curvature at every point, hence the **local** maximum
safe speed at every point:

```
v_max(s) = sqrt( a_lat / kappa(s) )        (kappa -> 0 => v_max -> inf)
```

`v_max(s)` is only the *local* limit ("if you were already turning at
constant curvature"). The **drivable profile** also respects the car's
longitudinal dynamics — two passes, take the minimum:

```
v_profile(s) = min(  v_max(s),        # local curvature limit (deadbanded)
                     v_brakeable(s),  # forward pass: can I brake for a
                                      # tighter curve within braking dist?
                     v_reachable(s) ) # backward pass: can I accelerate
                                      # here from a slower point behind?
```

This is the classic speed-profile solver (forward pass = braking,
backward pass = acceleration). It is **pure geometry, offline, no
traffic, no real time** — it runs once over the static map. Miss Daisy's
simulation run then *verifies* the profile is stable under the real
bicycle model (steering, traction); the solver plans, the simulation
proves.

**Worked example (a_lat = 2.0 m/s²):** R = 6 m → 12.5 km/h; R = 20 m →
23 km/h; R = 100 m → 51 km/h. Turn speed scales with the *square root*
of the radius — a 4× wider curve allows only 2× the speed. (Note: 50
km/h in a 6 m curve would need ~3.3 g — physically impossible; the
profile never produces that.)

### 9.4 Curvature deadband

If a road is truly straight but its OSM nodes carry slight noise, an
interpolating spline produces a tiny phantom S-curve. Guard: treat
κ below a small threshold (e.g. κ < 1/200 m⁻¹, i.e. "radius > 200 m")
as **straight** (κ = 0). The car brakes for real curvature, never for
noise. (Centripetal Catmull-Rom through near-collinear points already
yields near-zero κ, so this is belt and braces.)

### 9.5 Two levels of lateral acceleration (driver personality)

- **`a_lat_max` (vehicle capability):** what the car *can* physically
  sustain (e.g. ~8 m/s² ≈ 0.8 g, dry). A property of the vehicle.
- **`a_lat_style` (driver choice):** what the driver *uses*.
  Miss Daisy: 2.0 m/s² (cautious, comfortable). A sporty driver: 6–7
  m/s² (near the limit).

```
v_max(s) = sqrt( a_lat_style / kappa(s) ),   a_lat_style <= a_lat_max
```

**The line is sacred and identical for everyone; only the profile
scales per driver.** Miss Daisy authors the *geometry*; each driver's
*profile* is solved from their own `a_lat_style`. This is where the old
Phase-3 "driver personality" idea re-enters — as a profile parameter on
shared lines, not as a planning policy.

### 9.6 Authoring run

```
OSM (nodes + ways)
   -> smoothed geometry C(s) per way (centripetal Catmull-Rom, §10)
   -> kappa(s) at every point
   -> v_max(s) = sqrt(a_lat_style / kappa(s))        [with deadband]
   -> forward pass (brakeable) + backward pass (reachable)
   -> v_profile(s)                                   [Miss Daisy's line]
   -> bicycle-model verification run (on-road, stable)
   -> precomputed paths (line + profile)  ->  mass AI drivers follow
```

- Generated **at map load** (or shipped as a checked-in file; the map is
  static → author once, commit, and unit-test it: "every precomputed
  path stays on-road").
- The authoring run **is** the long-standing "test the bicycle model on
  the OSM map" item — real geometry (tight corners, slivers, one-way
  rings) for the first time.
- **Not precomputed** (stays runtime, local, cheap): reactions to other
  cars (brake/dodge; priority brake > dodge > hold, perception local via
  a spatial grid) and the live player (free mode has no reference line).
  Miss Daisy drives an empty map; her paths assume clear roads.

---

## 10. Smoothed geometry: the graph is sacred, the curve is a function

**Problem:** raw OSM geometry is a *polyline* — every connection between
consecutive nodes is a straight chord. Curves exist only as far as node
density allows. Consequences: κ = 0 on every chord and a kink (κ = ∞) at
every node → speed profiles see "straight road" across bends and never
brake (the roundabout chord-curvature bug, §7, was this in miniature);
pure pursuit gets kinked targets; paved polygon and driving line can
disagree.

**Principle (agreed): the graph is sacred; the curve is a function of the
graph.**

- The OSM node set and way topology are **untouched** — no new graph
  nodes, no way splits. The graph is the single source of truth for
  routing, junction logic, and the renderer.
- Each way's centerline becomes a **centripetal Catmull-Rom spline
  through its original OSM nodes** (interpolating: the curve passes
  *exactly* through every node — faithful to the data; C¹: no kinks).
  - *Why Catmull-Rom:* the tangent at a node follows the road's overall
    direction (from previous to next node) → the curve flows through the
    nodes, no zigzag, no overshoot. Local support (moving one node only
    affects its two neighboring pieces), cheap (cubic per piece),
    standard.
  - *Why centripetal:* OSM node spacing is irregular; the uniform variant
    can overshoot with uneven spacing, the centripetal variant cannot.
- **No inserted geometry.** The spline is stored as a *function defined by
  the original nodes*; position/heading/κ(s) are evaluated on demand. The
  only internal artifact is an arc-length lookup table (an evaluation
  *cache* of the math — it defines nothing, touches no graph node).
- **Junctions compose cleanly:** the spline ends at the junction node
  with a well-defined end tangent (direction last-but-one → last node).
  The 6 m corner fillet (`ROAD_CORNER_RADIUS_M`) connects the approach
  tangent to the exit tangent — tangent-continuous with the *splines*,
  not the old chords.
- **A road that is a chain of degree-2 nodes** (river through the
  mountains, pass road) is NOT rendered/driven as a zigzag: the spline
  reveals its true, continuous curvature — the S-bends are real geometry
  the polyline was hiding, and κ(s) tells us exactly how fast each bend
  allows (§9.3).

```
OSM way (nodes N0..Nk, irregular)
    -> centripetal Catmull-Rom (interpolating)
    -> smooth curve C(s) through N0..Nk, C1, no kinks
    -> position / heading / kappa(s)  on demand
         -> speed profile (§9.3)  · pure pursuit  · paved polygon
```

---

## 11. Rendering & debugging on the smoothed geometry (pygame)

### 11.1 One smoothed geometry, all consumers

Today each consumer (renderer, driving model, on-road check) builds its
own geometry from the raw chords — that is how they diverge. The plan:

```
OSM (nodes + ways)  --once at map load-->
    smoothed curves C(s) + kappa(s) + end tangents
         |
         +-> Renderer       (paved polygon, edges, markings, fillets)
         +-> Driving model  (reference line = C(s) + lane offset, profile)
         +-> On-road check  (is_car_on_road vs THE SAME paved polygon)
         +-> Miss Daisy     (authoring run on the same line)
```

The renderer draws exactly what the car drives; the on-road check tests
against exactly what is drawn. Divergence becomes structurally
impossible. (A small tolerance bridge covers the sub-cm difference
between the spline and its discretized polygon.)

### 11.2 Drawing curves in pygame

- Pygame has no native spline primitives → **dense resampling (every
  1–2 m) → `pygame.draw.polygon`** (Option A). Visually indistinguishable
  from a true curve (chord deviation < 1 cm at 2 m even in tight bends).
  The resampling is a *render cache*, not new geometry.
- Corner fillets: resampled uniformly like everything else (one code
  path; `pygame.draw.arc` exists but adds a special case for no visual
  gain).
- **Geometry lives in map space (X east, Y north, OSM convention).** The
  pygame transform (scale + **Y flip** + camera offset) is applied only
  at draw time. The renderer is a pure projection of the same geometry
  the driving model uses — they cannot drift apart.
- **Static map → pre-render once, blit per frame.** At map load: resample
  → draw the whole network onto one big `pygame.Surface`. Per frame:
  blit that surface at the camera offset + draw only the dynamic things
  (cars, blinkers, overlays). The expensive polygon drawing happens once,
  not 60×/s.

### 11.3 Drawn = drivable

We draw **exactly the drivable area** — nothing wider, no curbs, no
visual-only margin. ONE polygon, used by both `pygame.draw.polygon` and
`is_car_on_road`. Rationale: the eye can decide "in or out" instantly
and without interpretation; the drawn edge IS the hard edge. (A wider
visual road + inset drivable polygon is a possible later cosmetic
upgrade, not the default.)

### 11.4 Paint-bucket trail overlay ("dripping paint per tire")

A debug overlay that records where the car has been — making lane
discipline, corner cutting, oncoming-lane incursions, and off-road
excursions visible at a glance:

- **Data:** the four 4.5 m × 1.8 m car corners that `is_car_on_road`
  already computes — the trail shows *literally* what the validator
  checks. No new geometry, no divergence.
- **Layers per frame:** `map_surface` (static) → `trail_surface`
  (per-vehicle, accumulates small marks at the four corner positions)
  → current car + blinkers + overlays.
- **Memory:** persistent over the whole map surface first (the map is
  finite); switch to a ring buffer / fading trail (last ~300 m) if it
  gets too big.
- **Color-coded violation heatmap:**

  | state | color | meaning |
  |---|---|---|
  | in own lane | dark gray (rubber) | ok |
  | on the opposing lane | yellow | cut too wide |
  | outside the drivable area | red | on-road violation |

- **Toggle: global AND per-vehicle**, for ALL operating modes (AI mass,
  player assisted, player free). In free mode it is most valuable (you
  see instantly when your wrong-way dash crosses into oncoming traffic).
  Default on in dev, off in "clean" mode.

---

## 12. Traffic simulation: open system, one district

**Scope (agreed):** one district — **Kleinmachnow** — as the first (and
for now only) real test. Several hundred (~500) vehicles concurrently.

### 12.1 Exit points = boundary nodes of the graph

The OSM extract is finite. Where a road leaves the extract, it ends in a
**boundary node** (degree-1 node at the data edge; the road "continues"
beyond the data). These boundary nodes **are** the exit points —
computed once at map load:

```
exit_points = { node n | degree(n) == 1 and n lies at the extract edge }
```

Real dead ends *inside* the town are **not** exit points (the car stops/
turns back there); the distinction is geometric (at the data edge or
not).

### 12.2 Trip model: (origin, destination)

| origin | destination | behavior |
|---|---|---|
| point in town | point in town | internal trip (home → work, both inside); car drives there, then despawns (or takes the next trip) |
| point in town | **outside** | car drives to the **matching exit point** (the one whose exit direction best matches the destination direction) and **despawns there** |
| (optional) exit point | point in town | car **spawns** at the boundary (enters the district) and drives to the destination |

"Magical disappearance" at an exit point = **administrative removal**
from the simulation — no teleport physics, no crash, nothing rendered.
Example (agreed): someone drives from home (in town) to work (outside)
→ route to the exit point in the work direction → despawn.

### 12.3 Steady state

To measure jams and bottlenecks meaningfully the system must be in
**equilibrium**: roughly constant vehicle count (~500), constant inflow ≈
outflow.

- **Outflow:** despawns at exit points (and at internal destinations).
- **Inflow:** spawns at the matching rate — at the boundary (cars
  crossing the district) and/or at residential points in town (home →
  work trips).
- A **trip generator** draws (origin, destination) per car — a mix of
  internal and boundary-crossing trips — and couples spawn rate to
  despawn rate to hold the population stable.

### 12.4 Why only the bicycle model can do this

1. **Real gaps:** cars are points with position and speed → "distance to
   the car ahead" is a real quantity → jams emerge (each car brakes for
   the one ahead → the wave propagates back). That emergence IS the
   point of the simulation.
2. **Cheapness:** pure pursuit on a precomputed line is a few flops per
   car; 500 cars stay cheap because control is open-loop-ish and
   perception is local (spatial grid for neighbor lookup).
3. **Safety:** precomputed, Miss-Daisy-proven lines → no car teleports.
   The validator is a safety net, not a wall.

### 12.5 The payoff: metrics

Log per car over time: position, speed, headway, waiting time. From that:
**density, throughput, jam lengths, junction waiting times** — directly
answering the simulation's questions: where are the bottlenecks, where do
jams form, would closing road X improve the district? The analysis comes
from the *traffic*, not the graphics.

---

## 13. Onboarding for a new implementer (start here)

If you are an LLM (or human) picking up this project from this document
alone: read this section first, then §8 (decision + work order), then the
section for the specific task you are doing.

### 13.1 Where things stand (one paragraph)

The game is a top-down 2D car game (pygame + Flask REST API) on OSM road
data (default: Kleinmachnow). The old "rails" driving model (car = train
on the OSM graph, position = segment+offset) is **retired** (§8) — its
code still exists in `src/car.py` (`_update_position_rails`) but must not
be extended. A kinematic **bicycle model** is implemented and integrated
(`src/bicycle_nav.py`, `BicycleDriver` in `src/driver.py`, `--bicycle`
flag in `src/main.py`) but is **uncommitted** and has two known bugs
(§7 "Open — bicycle model"). The agreed direction: smooth the geometry
(§10), precompute drivable reference lines offline with a cautious
virtual driver ("Miss Daisy", §9), then run a ~500-car traffic
simulation of one district (§12) with two extra player modes (assisted,
free — §8) and a paint-bucket trail overlay (§11.4).

### 13.2 Hard rule (from AGENTS.md — read it)

Never produce physically impossible motion: no teleports, no instant
heading changes, no turning radius below the mechanical minimum. When a
desired behavior would require impossible motion, fix the underlying
logic — never weaken or bypass the physics validator.

### 13.3 Running things

```bash
# headless game + REST API (synthetic test map, deterministic)
SDL_VIDEODRIVER=dummy python -m src.main --map basic --api

# same, with the bicycle model
SDL_VIDEODRIVER=dummy python -m src.main --map basic --api --bicycle

# without --map: real OSM data (Kleinmachnow) — manual play / authoring

# test suite (game must be running from another terminal)
python tests/test_turning.py
python tests/test_turning.py --only <name> <direction> <speed>  # one scenario
python tests/test_api.py
```

Full workflow + REST endpoints: `docs/TESTING.md`, `docs/REST_API.md`.
Game spec: `docs/SPEC.md`. Synthetic test map (incl. the sliver-junction
layout, named start points like `sliver_approach`): `src/test_maps.py`.

### 13.4 Concept → code map

| plan concept | where it lives |
|---|---|
| bicycle model / reference line / pure pursuit | `src/bicycle_nav.py` (`BicycleNav`, `_offset_polyline_right`) |
| bicycle driver (AI) | `src/driver.py` (`BicycleDriver`) |
| retired rails model (do not extend) | `src/car.py` (`_update_position_rails`) |
| road network, paved polygon, on-road check | `src/road_network.py` (`is_car_on_road`) |
| config (car accel/top speed, corner radius, lane offset) | `src/config.py` |
| entry point, `--map` / `--bicycle` / `--api` flags | `src/main.py` |
| synthetic test map + named start points | `src/test_maps.py` |
| turning test suite (REST-driven) | `tests/test_turning.py` |
| standalone prototype (historical) | `scripts/proto_bicycle.py`; repro/debug scripts: `scripts/repro_crash.py`, `scripts/debug_handoff.py` |
| Miss Daisy authoring, smoothed geometry, trails, traffic sim | **not built yet** — §9/§10/§11.4/§12 |

### 13.5 Git state — read before touching anything

The working tree has **uncommitted bicycle-model work** (this is the
future; do not discard, revert, or "clean" it):

- modified: `src/car.py`, `src/config.py`, `src/driver.py`, `src/main.py`,
  `src/road_network.py`, `src/test_maps.py`, `tests/test_turning.py`,
  `docs/TURN_REWORK_PLAN.md`
- new (untracked): `src/bicycle_nav.py`, `scripts/`

Commit it per §7 ("Open — decision", item "Commit") — ideally after the
first work-order item below is done, so the commit contains a working
bicycle model.

### 13.6 First task, concretely (work-order item 1, §8.2)

**Fix the `sliver_approach` stall.**

- Symptom: on the synthetic map, teleport to start point
  `sliver_approach` and accelerate — the car is stuck at s=0 (speed 0.0):
  the reference line curves sharply at the sliver junction, the
  pure-pursuit target sits far off the car's heading, the steering angle
  is large, and the accel-scale gate (throttle off while steering hard)
  prevents any movement. It is a deadlock, not a slow start.
- Reproduce: `SDL_VIDEODRIVER=dummy python -m src.main --map basic --api
  --bicycle`, then drive it via the REST API as `tests/test_turning.py`
  does (teleport → blinker if needed → accelerate), or run the affected
  scenarios: `python tests/test_turning.py` (the 12 timed-out tests in
  the visible-window run include this one).
- Candidate fixes (from §7): (a) reduce the steering angle at the start
  (smaller base look-ahead, or a gentler effective corner radius at the
  sliver junction), or (b) allow some acceleration even while steering
  (lower the accel-scale gate's threshold). Pick whichever keeps the
  motion physically legal (§13.2) — verify with the validator enabled.
- **Done when:** the car leaves the sliver start point and completes the
  maneuver on-road; the 12 timed-out `tests/test_turning.py` scenarios
  pass; no `RuntimeError` from the physics validator; the existing green
  tests stay green (the 3 pre-existing failures in `test_api.py` /
  `test_random_road_point` are unrelated and remain).
