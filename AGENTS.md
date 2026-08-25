# Project Rules

## Debugging workflow: reproduce first, fix second

When the user reports a problem (visual glitch, wrong behavior, stutter,
broken maneuver): **do not start designing or writing code changes.**
First step is always:

1. **Reproduce** it deterministically (headless sim, `--only`/`--tests`
   single-scenario run, or a scripted REST drive on the live game).
2. **Log** what actually happens (state trace: speed, position, heading,
   blinkers, segments over time) at high enough resolution to see the
   failure.
3. **Understand** the root cause from the data - only then discuss or
   implement a fix.

No speculative fixes based on guessing what the user saw.

## General rule: never do physically impossible things

The simulation must never produce motion that no real car could perform:
no teleportation, no instant heading changes, no turning radius below a
real car's mechanical minimum, no position/heading updates that fall out
of sync. This applies to game logic, physics, fallbacks, and edge cases
(junctions, dead ends, segment hand-offs) alike.

When a desired behavior would require physically impossible motion, fix
the underlying logic (e.g. brake earlier, blend offsets, smooth the
heading) instead of weakening or bypassing the checks that catch it.

This rule may ONLY be violated if the user explicitly asks for it.

## Decisions: explicit user decisions are binding

When the user makes an explicit decision or instruction (e.g. "this value
must be variable, not fixed"), it is **binding**. You may propose
alternatives and point out consequences, but you must NOT silently override
the decision with your own judgment ("actually, simpler: keep it fixed").
If you believe the decision has a problem, state the conflict explicitly
and ask before deviating. Acknowledging a correction and then implementing
the old way anyway is a violation of this rule.

## E2E tests

The test suite in `tests/test_turning.py` is the project's **end-to-end
(e2e) test**: it drives the live game (real physics, real rendering, real
road network) over the REST API. See `docs/TESTING.md` for the full
workflow.

When the user asks to run the e2e tests:

0. **Restart the game first.** Kill any running game process
   (`pkill -f "src.main"`) and start a fresh one - no stale state from
   previous runs or debugging: `--map basic --api`. For a single-scenario
   run add `--start <name>` (the scenario's start point, e.g.
   corner_right_entry for test 1): on test maps this only FOCUSES THE
   CAMERA there at driving zoom - no car is spawned - so the window shows
   the scenario from frame one and the suite's setup teleport places the
   car exactly where the view is looking. Without --start the view opens
   at world centre; either way the camera snaps to the car on teleport.
1. **Map visible.** Start the game WITHOUT `SDL_VIDEODRIVER=dummy` - the
   game window (with the map) must be open so the user can watch the car
   drive. (`SDL_VIDEODRIVER=dummy` is only for CI or when the user
   explicitly asks for it - see `docs/TESTING.md`.)
2. **Results visible while running.** Run the suite in the foreground,
   with output going to the console AND to a persistent log via `tee`:

   ```bash
   python tests/test_turning.py 2>&1 | tee /tmp/test_run.log
   ```

   The user must see every test's output and the pass/fail summary as it
   happens. No backgrounding, no `nohup`, no discarding output. The `tee`
   file lets the log be re-read or grepped afterwards (long runs can
   exceed the tool output limit).

(The game process itself may run in the background; only the *test run*
has to be visible.)
