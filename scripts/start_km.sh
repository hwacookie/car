#!/bin/bash
# Start the Kleinmachnow OSM map (src.main without --map), killing any
# running game instances first. Waits until the REST API answers.
set -e
cd "$(dirname "$0")/.."

echo "Killing running game instances..."
pkill -f "src.main" 2>/dev/null || true
sleep 1

nohup .venv/bin/python -u -m src.main --api > /tmp/game_km.log 2>&1 &
GAME_PID=$!
echo "$GAME_PID" > /tmp/game_km.pid
echo "Started game (pid $GAME_PID), log: /tmp/game_km.log"

# Wait for the REST API (map load included) - up to 60 s.
for i in $(seq 1 60); do
    if curl -s localhost:5000/health >/dev/null 2>&1; then
        echo "Kleinmachnow game is up (pid $GAME_PID), API on http://127.0.0.1:5000"
        exit 0
    fi
    sleep 1
done

echo "Game did not come up - last log lines:" >&2
tail -10 /tmp/game_km.log >&2
exit 1
