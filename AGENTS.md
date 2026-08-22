# Project Rules

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

## E2E tests

The test suite in `tests/test_turning.py` is the project's **end-to-end
(e2e) test**: it drives the live game (real physics, real rendering, real
road network) over the REST API. See `docs/TESTING.md` for the full
workflow.

When the user asks to run the e2e tests:

0. **Restart the game first.** Kill any running game process
   (`pkill -f "src.main"`) and start a fresh one - no stale state from
   previous runs or debugging.
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
