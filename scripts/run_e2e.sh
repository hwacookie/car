#!/usr/bin/env bash
# Run the e2e test suite (tests/test_turning.py) against a fresh game instance.
#
# Workflow (see docs/TESTING.md):
#   1. Kill any stale game process
#   2. Start the game with the test map + REST API, window VISIBLE
#      (no SDL_VIDEODRIVER=dummy - you should be able to watch the car)
#   3. Wait until the API is healthy
#   4. Run the suite in the foreground: output goes to the console
#      AND to /tmp/test_run.log via tee
#   5. Stop the game again on exit
#
# Usage:
#   scripts/run_e2e.sh                                  # full suite
#   scripts/run_e2e.sh --only corner_right_entry right 80   # single scenario
#
# Logs:
#   game process:  /tmp/game.log
#   test output:   /tmp/test_run.log
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

GAME_LOG=/tmp/game.log
TEST_LOG=/tmp/test_run.log
API_URL=http://localhost:5000/health

echo "==> Killing any running game..."
pkill -f "src.main" || true
sleep 1

echo "==> Starting game (visible window) -> $GAME_LOG"
nohup python -m src.main --map basic --api > "$GAME_LOG" 2>&1 &
GAME_PID=$!

cleanup() {
  echo "==> Stopping game (pid $GAME_PID)..."
  kill "$GAME_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Waiting for API at $API_URL ..."
ready=0
for i in $(seq 1 30); do
  if curl -sf "$API_URL" > /dev/null 2>&1; then
    echo "==> API ready after ${i}s"
    ready=1
    break
  fi
  if ! kill -0 "$GAME_PID" 2>/dev/null; then
    echo "!! Game process died during startup. Last log lines:"
    tail -30 "$GAME_LOG"
    exit 1
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "!! API never came up. Last log lines:"
  tail -30 "$GAME_LOG"
  exit 1
fi

echo "==> Running e2e suite (log: $TEST_LOG)..."
# -u: unbuffered stdout, so output appears on the console immediately
#     (piping through tee would otherwise block-buffer it until the end)
python -u tests/test_turning.py "$@" 2>&1 | tee "$TEST_LOG"
TEST_EXIT=${PIPESTATUS[0]}

# Check if game is still alive; if not, distinguish crash vs normal exit
if ! kill -0 "$GAME_PID" 2>/dev/null; then
  if grep -qiE "Traceback|Error|Exception" "$GAME_LOG" 2>/dev/null; then
    echo ""
    echo "!! Game process CRASHED (stack trace in log). Last lines:"
    tail -20 "$GAME_LOG"
  else
    echo ""
    echo "==> Game exited normally (closed by user or finished)."
  fi
fi

exit $TEST_EXIT
TEST_EXIT=${PIPESTATUS[0]}

# Check if game is still alive; if not, distinguish crash vs normal exit
if ! kill -0 "$GAME_PID" 2>/dev/null; then
  if grep -qiE "Traceback|Error|Exception" "$GAME_LOG" 2>/dev/null; then
    echo ""
    echo "!! Game process CRASHED (stack trace in log). Last lines:"
    tail -20 "$GAME_LOG"
  else
    echo ""
    echo "==> Game exited normally (closed by user or finished)."
  fi
fi

exit $TEST_EXIT
