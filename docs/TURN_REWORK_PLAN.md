# Turn-Planning Rework — Plan

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

1. Phase 1 (1–3) — smallest diff, kills the crash loop immediately.
2. Phase 2 (4–7) — the real fix; the no-plan fallback becomes rare.
3. Phase 3 (8–11) — policy extraction; behavior-preserving refactor,
   sporty style as a stub.
