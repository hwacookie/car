# Session state — turn rework (compacted chat snapshot)

Saved: 2026-07-08. This is the verbatim compacted conversation summary from the
pi session working on `docs/TURN_REWORK_PLAN.md`, kept so the full context
survives chat compaction. Update the "In Progress / Next Steps" sections as
work proceeds.

## Goal
- Read and understand `docs/TURN_REWORK_PLAN.md`.
- Explain the `sliver_approach` scenario and the current stall bug.
- Continue with the plan's next task: fix the `sliver_approach` stall and the related timed-out tests.

## Constraints & Preferences
- Hard rule from `AGENTS.md`: never do physically impossible things.
- Fix the logic; do not weaken the validator.
- Preserve the exact project paths and scenario names.
- The bicycle model is the intended physics for all modes; the rail model is retired.
- The user asked for an explanation in German, but the working summary can remain in English.
- Use the project venv Python for game/repro/test runs: `.venv/bin/python`.
- Standalone replay must match teleport behavior, including `progress = 0.0` and `_apply_plain_segment_position()`.
- Reference-line geometry must remain physically feasible for the bicycle model: the car's minimum turning radius is `WHEELBASE / tan(MAX_STEER) = 2.7 / tan(38°) ≈ 3.46 m`.
- The generic corner-rounding helper is shared with rendering, so reference-line-specific fixes should not blindly change rendering fillet behavior.
- The running game must be restarted after code changes because the test suite communicates with the live game API.
- The game process is started as `python -m src.main --map basic --api --bicycle`, so `pkill -f "src.main"` is required; `pkill -f "src/main.py"` is not sufficient.
- macOS does not provide `timeout` by default; test runs should rely on the test suite's internal timeout.
- Off-road validation uses the rendered paved polygon, not a simple rectangle/circle approximation.
- Python stdout is block-buffered when redirected to a file; use `python -u` for real-time monitoring of test output.

## Progress
### Done
- [x] Read `docs/TURN_REWORK_PLAN.md`.
- [x] Summarized current project status:
  - Rail model retired.
  - Bicycle model is the unified physics for AI mass, player-assisted, and player-free modes.
  - Bicycle model work exists in the working tree but is **not committed**.
  - Two known blockers:
    - `sliver_approach` stall.
    - 12 timed-out tests in `tests/test_turning.py`.
- [x] Explained the `sliver_approach` geometry and bug.
- [x] Ran the `left` reproduction with the venv — PASSED.
- [x] Verified actual test behavior for `sliver_approach left` — PASSED.
- [x] Verified actual test behavior for `sliver_approach right` before fix — TIMEOUT.
- [x] Ran the `right` reproduction — confirmed deadlock at v=0.
- [x] Inspected key project structures: `src/test_maps.py`, `src/road_network.py`, `src/car.py`, `src/driver.py`, `src/main.py`, `src/bicycle_nav.py`, `src/config.py`.
- [x] Dumped sliver segment indices and start points.
- [x] Confirmed `sliv_w` is a dead-end node (degree 1).
- [x] Created standalone debug script: `/Users/hauke/prj/car/scripts/debug_sliver.py`.
- [x] Fixed debug script pixel/meter scale: `PPPM = 2.0`.
- [x] Ran standalone debug for `right` — confirmed deadlock loop.
- [x] Dumped corrected right-turn reference-line geometry.
- [x] Identified and corrected debugging mistake (progress=0.5 vs 0.0).
- [x] Read and analyzed `_round_polyline_corners` in `src/road_network.py`.
- [x] Computed car's mechanical minimum turning radius: `3.46 m`.
- [x] Compared left/right/straight sliver reference lines.
- [x] Read the speed-profile logic in `src/bicycle_nav.py`.
- [x] Read `BicycleNav` constants and route-building context.
- [x] Read `monitor_turn` in `tests/test_turning.py`.
- [x] Simulated a standstill creep-floor fix — confirmed it breaks the deadlock.
- [x] Identified two interacting root causes for sliver stall.
- [x] Investigated `sliver_from_east` route construction — anomalous delta remains.
- [x] Investigated `sliver_from_west` route construction.
- [x] Read lane-offset logic in `src/bicycle_nav.py`.
- [x] Analyzed possible minimum-radius corner fixes.
- [x] Decided to implement the standstill creep fix first.
- [x] Implemented creep fix in `/Users/hauke/prj/car/src/bicycle_nav.py`:
  - `CREEP_SPEED = 1.0`, `CREEP_SCALE = 0.3`
  - `if accel and car.speed < self.CREEP_SPEED: accel_scale = max(accel_scale, self.CREEP_SCALE)`
- [x] Restarted the game to load the fix.
- [x] Ran the actual right-turn test after the creep fix — PASSED.
- [x] Enhanced standalone debug script with off-road check.
- [x] Read `is_on_road` implementation in `src/road_network.py:284`.
- [x] Ran enhanced standalone debug for right turn (post-creep-fix) — CLEAN DYNAMICS.
- [x] Verified all three sliver_approach tests pass after creep fix.
- [x] Identified the deterministic test suite structure (18 tests).
- [x] Ran full deterministic test suite:
  - Tests 1–8 (ALL TURNS): TIMEOUT
  - Test 9 (`crossroads_from_north` straight): PASSED
  - Test 10 (`oneway_entry` straight): PASSED
- [x] Investigated test 1 (`corner_right_entry` right) timeout in detail:
  - Car goes straight, never turns (max heading change 0.0°)
  - Expected end segment = 1 = initial segment (contradictory test logic)
- [x] Read `monitor_turn` pass/fail logic in detail — confirmed contradictory condition.
- [x] Confirmed game is running fixed code during full suite.
- [x] **Confirmed turn test failures are PRE-EXISTING (not a regression)**:
  - Temporarily disabled creep fix (commented out the `CREEP_SPEED`/`CREEP_SCALE` lines)
  - Restarted game without creep fix
  - Ran `corner_right_entry right` test: STILL TIMED OUT
  - Same behavior: car goes straight, 56 km/h, 0.0° heading change
  - Restored creep fix from `/tmp/bicycle_nav_with_creep.py`
- [x] **Created general debug script**: `/Users/hauke/prj/car/scripts/debug_turn.py`
  - Usage: `.venv/bin/python scripts/debug_turn.py <start_point> <direction> [seconds]`
  - Replays game flow without pygame/Flask
  - Prints nav internal state every 0.5s
  - Dumps initial reference line geometry
- [x] **Identified root cause of turn test timeouts (tests 1–8)**:
  - The corner junction is 250m away (seg 1 length = 250m)
  - Car accelerates to ~28 m/s (100 km/h) over first 10s
  - Speed profile drops car to **3.55 m/s** (crawling) from s=190 to s=250
  - Car crawls at 3.55 m/s for 60m before reaching the corner
  - Car does NOT reach the corner within the 15s monitoring window
  - The car IS turning eventually (delta starts increasing at s≈245) but too slowly
- [x] **Dumped speed profile for corner_right_entry**:
  - Route: `['cornerR_n', 'cornerR_c', 'cornerR_w']`
  - ref_total: 494.67 m
  - s=0–30: v_tgt = 55.60 m/s (V_MAX)
  - s=30–190: v_tgt drops from 55.60 → 3.55 m/s
  - s=190–250: v_tgt = 3.55 m/s (FLAT crawling for 60m)
  - s=250–260: v_tgt jumps back to 55.60 m/s
  - s=260–340: v_tgt = 55.60 m/s
  - s=340–490: v_tgt drops to 0 (stopping at end of route)
- [x] **Dumped reference line geometry for corner_right_entry**:
  - s=0 to s=240: perfectly straight south (x=1696.5, heading=180°)
  - s=245: heading starts changing (-160.5°)
  - s=250: heading -93.1° (fillet center)
  - s=255+: heading -90° (west, on seg 2)
  - Fillet occupies only ~10m of arc (s≈245–255)
  - The 60m crawling zone (s=190–250) is on STRAIGHT road — the speed profile is applying the corner speed cap too far back
- [x] **Confirmed test file changes via git diff**:
  - `corner_right_entry right`: expected seg changed 2 → 1
  - `tjunction_from_top left`: 6 → 7
  - `tjunction_from_top right`: 7 → 5
  - `y_from_stem left`: 9 → 10
  - `y_from_stem right`: 10 → 9
  - `crossroads_from_north left`: 13 → 14
  - `crossroads_from_north right`: 14 → 13
  - Comment: "The expected end segments below were re-verified against actual bicycle-mode runs on the new map."
  - Sliver tests added: `sliver_approach` straight/right/left
- [x] **Confirmed segment geometry for corner**:
  - seg 1: `cornerR_n -> cornerR_c`, (1700,700)→(1700,200), len=250m (approach south)
  - seg 2: `cornerR_c -> cornerR_w`, (1700,200)→(1200,200), len=250m (right exit west)
  - seg 3: `cornerL_n -> cornerL_c`, (2200,700)→(2200,200), len=250m
  - seg 4: `cornerL_c -> cornerL_e`, (2200,200)→(2700,200), len=250m
  - seg 5: `tjunc_top -> tjunc_center`, (3500,700)→(3500,200), len=250m
  - seg 6: `tjunc_center -> tjunc_w`, (3500,200)→(3160,200), len=170m
  - seg 7: `tjunc_center -> tjunc_e`, (3500,200)→(3840,200), len=170m

### In Progress
- [x] **Investigate the unexpected `sliver_from_east` spawn steering demand** — RESOLVED:
  - Re-ran the route + reference-line construction for all three turns (straight/left/right) in the current code.
  - All produce SANE routing and small steering demands: straight→sliv_w (delta@4m +23.6°), left→sliv_str (+12.3°), right→sliv_ap (+23.6°). The +~24° is just the car merging from the centerline onto the 1.75 m right-offset lane — expected.
  - The old `-81.9°` is NOT reproducible; it was cleared by the route-building / speed-profile work. No further action needed.
  - NOTE: `sliver_from_east` is NOT in the deterministic 18-test suite (only `sliver_approach` straight/right/left are), so this never blocked the suite.
- [ ] **Decide whether to add a navigation-specific corner-radius feasibility fix** for the sliver case:
  - The sliver right-turn reference line requests a fillet radius of ~2.06 m < the 3.46 m mechanical minimum.
  - The car handles it by cutting the corner at its 3.46 m limit while creeping; the test passes because the off-road check is against the paved polygon (with width tolerance).
  - The car's ACTUAL motion is physically feasible (max steer, no impossible motion); only the idealized reference line is infeasible.
  - Fixing it properly would mean making the sliver fillet >= 3.46 m in navigation without changing the shared rendering fillet. Deferred — tests pass, motion is physical.

### Blocked
- (nothing — all 18 tests pass, and the bicycle-model work is committed as `2066311`)

## Key Decisions
- **Bicycle model is THE physics**: Rail model retired.
- **Next task is fixing `sliver_approach`**: Immediate blocker before smoothed geometry, Miss Daisy, rendering, traffic sim.
- **Fix must remain physically plausible**: No impossible motion or weakened validation.
- **Option (b) preferred**: Allow limited creep while steering from standstill.
- **The `right` case is the hard blocker**: `left` already passes; `right` was a complete standstill deadlock.
- **Standalone debug must mirror teleport exactly**: `progress = 0.0` and `_apply_plain_segment_position()`.
- **`PPPM` is 2, not 10**: All standalone geometry calculations use `PIXELS_PER_METER = 2`.
- **The accel gate must not deadlock at standstill**: A real driver can creep forward while steering hard.
- **The reference line asks for an impossible turn in sliver case**: Clamped fillet radius 2.06m < 3.46m minimum.
- **A creep floor is a useful minimal fix, but likely not the full fix** for sliver.
- **Implemented creep floor before changing corner geometry**: Minimal, physically motivated, doesn't alter shared rendering.
- **The sliver_approach fix is validated**: All three sliver tests pass, clean dynamics confirmed.
- **Turn test failures are PRE-EXISTING, not a regression**: Confirmed by disabling creep fix and re-running — same timeout.
- **The turn test root cause is the speed profile, not the steering**: The car goes straight because it's crawling at 3.55 m/s due to an overly aggressive braking ramp. The reference line IS planning the turn (fillet at s≈245–255), but the car can't reach it in time.
- **The speed profile's 60m curvature lookahead is too aggressive**: It applies the corner speed cap 55m before the fillet starts, on perfectly straight road. This causes 60m of crawling that makes the car miss the 15s monitoring window.
- **Test configuration for `corner_right_entry` is contradictory**: `expected_end_segment = 1 = initial_segment`. The test logic requires the segment to CHANGE from initial, so this can never pass. Need to determine if the expected segment should be 2 (the right exit) or if the test logic needs adjustment.
- **Do not modify shared corner rounding yet**: `_round_polyline_corners` is used by rendering and navigation.
- **The creep fix is NOT the cause of the full-suite turn timeouts**: Sliver tests pass; turn failures show car going straight (speed profile issue), not stalling.

## Next Steps
1. [x] **Commit the bicycle-model work** — DONE: commit `2066311` on branch `turn-planning-rework` (16 files, +3275/-258). The only remaining uncommitted change is the unrelated cosmetic `.vscode/settings.json` theme tweak (intentionally left out).
2. [x] **Investigate `sliver_from_east` anomaly** — RESOLVED (see In Progress; no longer reproducible).
3. **Decide whether to add a navigation-specific corner-radius feasibility fix** for the sliver case (see In Progress). Deferred — tests pass, motion is physical.
4. **Continue with the plan's next phase** (all 18 tests now pass):
   - Smoothed geometry in §10
   - Miss Daisy offline reference-line authoring in §9
   - Pygame rendering / paint-bucket trails in §11
   - ~500-car Kleinmachnow traffic simulation in §12

## Critical Context
- Relevant files:
  - `docs/TURN_REWORK_PLAN.md`
  - `src/bicycle_nav.py` (contains the speed profile logic to fix)
  - `src/test_maps.py`
  - `src/road_network.py`
  - `src/car.py`
  - `src/driver.py`
  - `src/main.py`
  - `src/config.py`
  - `tests/test_turning.py`
  - `scripts/repro_sliver_stall.py`
  - `scripts/debug_sliver.py`
  - `scripts/debug_turn.py` (new — general debug for any start point)
- **Speed profile bug details** (the current blocker):
  - For `corner_right_entry right`:
    - Route: `['cornerR_n', 'cornerR_c', 'cornerR_w']`, total 494.67m
    - Reference line is STRAIGHT from s=0 to s=240 (x=1696.5, heading=180°)
    - Fillet at s≈245–255 (heading changes from 180° to -90°)
    - Speed profile: V_MAX (55.60) until s=30, then drops to 3.55 by s=190
    - 3.55 m/s flat from s=190 to s=250 (60m of crawling on STRAIGHT road)
    - Jumps back to 55.60 at s=260 (after the fillet)
    - The 60m curvature lookahead window is causing the corner speed to be applied too early
  - Car speed trajectory: 0→28 m/s (accelerating, s=0–140), then 28→3.55 m/s (braking, s=140–190), then 3.55 m/s flat (crawling, s=190–250)
  - In 15s monitoring, car only reaches s≈193 (213m total travel), well short of the 250m corner
- **Corner geometry**:
  - seg 1: `cornerR_n -> cornerR_c`, (1700,700)→(1700,200), 250m south
  - seg 2: `cornerR_c -> cornerR_w`, (1700,200)→(1200,200), 250m west
  - Start point `corner_right_entry`: (1700, 700), heading 180°, seg 1, forward=True
- **Test configuration issue**:
  - `corner_right_entry right` has `expected_end_segment = 1` (same as initial)
  - Test logic: `if state['segment'] != initial_segment: if state['segment'] == expected_end_segment: reached = True`
  - This is contradictory when initial == expected
  - Git diff shows it was changed from 2 → 1 in the working tree
- **Sliver junction geometry** (tile `(0,3)`):
  - `sliv_ap` → `sliv_junc` (4.22m), `sliv_junc` → `sliv_str` (4.99m), `sliv_w` → `sliv_junc` (36.6m), `sliv_junc` → `sliv_e` (19.79m)
- **BicycleNav constants**:
  - `WHEELBASE = 2.7`, `CAR_LENGTH_M = 4.5`, `MAX_STEER = 38°`
  - `A_LAT_MAX = 2.0`, `A_BRAKE = config.CAR_BRAKING`, `V_MAX = config.CAR_SPEED`
  - `CORNER_RADIUS_M = 6.0`, `HORIZON_SEGMENTS = 6`
  - `CREEP_SPEED = 1.0`, `CREEP_SCALE = 0.3` (the fix)
  - Lane offset: 1.75m right/straight, 0.875m left
- **Game process management**:
  - Kill: `pkill -f "src.main"`
  - Start: `SDL_VIDEODRIVER=dummy nohup .venv/bin/python -m src.main --map basic --api --bicycle > /tmp/game.log 2>&1 &`
  - Health check: `curl -s http://localhost:5000/health`
  - State: `curl -s http://localhost:5000/state`
  - Start points: `curl -s http://localhost:5000/start_points`

---

## Post-compaction addendum (2026-07-08, after re-compaction)

Findings made after the snapshot above was taken:

- Read `_build_speed_profile` in full (`src/bicycle_nav.py:379`). Confirmed the
  mechanism: `LOOKAHEAD_M = 60.0` — for every arc-length point the code takes
  the MAX curvature over the next 60 m and hard-caps `profile[i]` at
  `sqrt(A_LAT_MAX / k_max)`. For the 90° corner the fillet curvature
  (R≈6.3 m → k≈0.159 1/m) is therefore applied as a hard 3.55 m/s cap from
  ~60 m before the fillet. The backward forward-reachability pass
  (`v[i] = min(v[i], sqrt(v[i+1]² + 2·A_BRAKE·d))`) already builds a correct,
  smooth braking ramp from the corner speed — the 60 m lookahead is redundant
  for sharp corners and harmful.
- The lookahead exists for the roundabout ring: 64 chords × 9.8 m (ring radius
  ≈ 100 m). `RefLine.curvature_at` uses a window `h = max(1.0, total·0.01)`
  (≈2 m on the 206 m roundabout route), so local curvature reads **0 on chord
  middles** and ≈0.0238 1/m (R≈42 m) only at chord junctions. A pure local
  cap would let the car run the ring at cruise between junctions.
- Config values confirmed: `CAR_SPEED = 55.6`, `CAR_ACCELERATION = 2.8`,
  `CAR_BRAKING = 10.0`, `CAR_LENGTH = 4.4`, `ROAD_CORNER_RADIUS_M = 6.0`.
- Braking math: from 55.6 m/s to 3.55 m/s at 10 m/s² needs ≈154 m — far more
  than the 60 m the old code allowed, i.e. the old profile was infeasible.
- **Change in flight (UNVERIFIED)**: replaced the 60 m lookahead block with a
  pure LOCAL curvature cap in `_build_speed_profile`. Backup of the
  pre-change file: `/tmp/bicycle_nav_backup.py`. The verification run for
  `corner_right_entry` was cut off by an output token limit and has NOT been
  run yet.
- Open design question: pure local cap may be too permissive on the
  roundabout ring (0 curvature on chord middles). Candidate: a short
  (~20 m) smoothing window for the cap — but a hard step where the window
  starts catching the fillet creates an infeasible instantaneous deceleration,
  so the windowed cap must blend smoothly (or rely on the braking pass).
  Needs testing on BOTH `corner_right_entry` and `roundabout_from_north`.

## Runtime observation #1: black pixels under the car — DIAGNOSED & FIXED (2026-07-13)

**Symptom:** User reported pure-black (0,0,0) pixels on the road surface just
below-right of the car's rear, moving with the car.

**Diagnosis:**
- The game code draws NO black anywhere (searched all of src/).
- Headless render of the exact live state showed no black (dummy driver
  composites the fringe differently than the live display).
- Root cause: the car sprite `assets/car_64x128.png` contained **136 pure-black
  (0,0,0) anti-aliasing pixels** at the body edges (bbox x:7-57, y:32-121),
  concentrated at the bottom-right rear corner. When pygame scales (64x128 ->
  ~22x54) and rotates the sprite, these black edge pixels leak through as a
  black fringe ("premultiplied-alpha black fringe" — a known pygame scaling
  artifact). On the user's display this renders as visible pure black on the
  road at the car's rear edge.

**Fix:** Surgical sprite cleanup — for every sprite pixel with alpha>0 and
r,g,b all < 15 (the black anti-aliasing fringe), replaced RGB with the car's
red (205,41,41), keeping alpha. 141 pixels fixed. The legitimate dark
windshield (70,81,86), wheels, and side shading are UNTOUCHED (they're > 15).
Result: 0 near-black fringe pixels remain; red body (4041 px), dark windshield
(966 px), white highlight (33 px) all intact.

**Verify:** Restart the game (`python -m src.main --map basic --api --bicycle`)
and look at the car's rear edge — the black fringe should be gone. The car edge
now anti-aliases to red instead of black.

**Note:** This is a sprite-asset fix, not a code change. If the black reappears
or other dark fringes show, the broader fix would be to regenerate the sprite
with proper red (not black) edge anti-aliasing.

---

## Session addendum (2026-08-19): rails-removal regression — found & fixed

The full status now lives in `docs/TURN_REWORK_PLAN.md` §0 ("Where we stand /
What we did / What's still missing"). Summary of this session:

- **Context:** commit `74b9c4e` ("Remove RAILS mode") turned all 18
deterministic tests red (2 off-road, 14 timeout, 2 wrong-end-segment), even
though the rails model was genuinely unused (the bicycle model was active).
- **Root cause #1 (the real one):** the rails cleanup deleted
  `_apply_plain_segment_position()` from `teleport_to_named_point()` without a
  replacement, so named-point teleports updated heading/segment/progress/speed
  but **never set `self.x`/`self.y`** — the car stayed put and drove off-road
  from wherever it happened to be. Fixed in `9cfcf10` (2-line addition).
- **Root cause #2 (a red herring):** the same commit had **bundled in the §10
  smoothed-geometry pipeline** (`SmoothCurve` / centripetal Catmull-Rom) by
  replacing `RefLine`'s chord polyline with a spline. The spline was NOT the
  cause of the failures — reverting it alone turned the failures into 18
  off-road (the teleport bug was still there). Reverted in `33cb300` so §10
  stays a standalone task.
- **Also enabled by `74b9c4e`:** the off-road check is now run for ALL modes
  (it was RAILS-mode-only at `2066311`), which is why the off-road behavior is
  now visible. The check is correct; the car genuinely needed the teleport fix.
- **Result:** 18/18 deterministic tests pass (0 off-road, 0 snaps, 0 timeouts,
  0 wrong segment).
- **Other this session:** solid-green background (dropped `make_grass_background`),
  tests pinned to `127.0.0.1` (macOS ControlCenter squats on `localhost:5000` /
  `::1`), road-signs feature designed + added to §7, and
  `scripts/visualize_junction_fillets.py` (circular vs Bézier junction
  connections; found a single cubic Bézier overshoots peak curvature vs the
  circular arc for turn angles >~100° → chain two for large angles).
- **Next task:** §10 smoothed geometry, re-introduced deliberately (see §0 / §8.2).
  Known gotcha: `SmoothCurve`'s `kap` table is corrupted at every piece
  junction (duplicate sample → κ spikes up to ~16 1/m); use geometry-based
  curvature (central differences of `point_at`) for any curvature measurement.
