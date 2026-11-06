#!/usr/bin/env bash
# Run the full robot code in the headless physics sim with the elevator E2E driver enabled
# (frc.robot.util.SimE2E). Exits with the robot's exit code; fails after a hard timeout.
# Usage: tools/elevator/run_e2e_sim.sh <log_dir>
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
logdir="$(mkdir -p "$1" && cd "$1" && pwd)"
timeout_s="${E2E_TIMEOUT_SECONDS:-240}"
out="$logdir/sim_console.txt"

ELEVATOR_E2E_LOG_DIR="$logdir" ./gradlew simulateJava --offline >"$out" 2>&1 &
pid=$!
( sleep "$timeout_s"; echo "[run_e2e_sim] timeout after ${timeout_s}s" >>"$out"; pkill -P $pid; kill $pid ) 2>/dev/null &
watchdog=$!
set +e
wait $pid
code=$?
set -e
kill $watchdog 2>/dev/null || true
wait $watchdog 2>/dev/null || true
grep -E '\[SimE2E\]|Exception|Error|overrun' "$out" | head -20 || true
echo "[run_e2e_sim] exit $code, console: $out"
exit $code
