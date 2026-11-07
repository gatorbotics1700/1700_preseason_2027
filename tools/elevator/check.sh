#!/usr/bin/env bash
# Pass condition for the elevator + 3D sim task. Exit 0 == task passes. Nothing else counts.
#
#   C1  ./gradlew build exits 0 (compile + spotlessCheck + unit tests, tests actually enabled)
#   C2  JUnit results: required elevator test classes ran, 0 failures / errors / skips
#   C3  Generated CAD model (ascope_assets/) is byte-identical to a fresh regeneration
#       from ElevatorConstants (CAD and code share one source of truth)
#   C4  Headless end-to-end sim run of the real robot code exits 0 and writes a .wpilog
#   C5  Log checker: elevator homes, reaches every setpoint within tolerance and time,
#       stays inside travel limits, and logged 3D component poses match the kinematics
#   C6  AdvantageScope layout JSON parses and references the elevator component poses
#   C7  Every changed file (vs main) is inside the plan allowlist
#   C8  No per-loop leaks in the E2E run (loop time flat, HubTracker logs only on changes)
#   C9  No CTRE device reports the same CAN error twice (dead or nonexistent devices)
#
# Usage: tools/elevator/check.sh            (all checks)
#        SKIP_E2E=1 tools/elevator/check.sh (C1-C3, C6, C7 only; for fast unit loops)
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

pass() { echo "PASS $1"; }
fail() { echo "FAIL $1: $2" >&2; exit 1; }

# ---- C1 -------------------------------------------------------------------------------
buildlog="$(mktemp)"
before="$(git diff -- src | shasum)"
./gradlew build --offline -q >"$buildlog" 2>&1 \
  || { tail -40 "$buildlog" >&2; fail C1 "gradle build failed"; }
# build runs spotlessApply first; the tree must already be formatted
[[ "$before" == "$(git diff -- src | shasum)" ]] || fail C1 "spotless reformatted files during build (run ./gradlew spotlessApply and keep the result)"
pass C1

# ---- C2 -------------------------------------------------------------------------------
python3 -I tools/elevator/check_junit.py build/test-results/test \
  frc.robot.subsystems.elevator.ElevatorKinematicsTest \
  frc.robot.subsystems.elevator.ElevatorPhysicsTest \
  frc.robot.subsystems.elevator.ElevatorModelTest \
  || fail C2 "junit results"
pass C2

# ---- C3 -------------------------------------------------------------------------------
python3 -I tools/elevator/generate_model.py --check || fail C3 "model out of sync with constants"
pass C3

# ---- C4 + C5 --------------------------------------------------------------------------
if [[ "${SKIP_E2E:-0}" != "1" ]]; then
  logdir="logs/elevator_e2e"
  rm -rf "$logdir"
  marker="$(mktemp)"
  tools/elevator/run_e2e_sim.sh "$logdir" || fail C4 "headless sim run failed"
  log="$(ls -t "$logdir"/*.wpilog 2>/dev/null | head -1 || true)"
  [[ -n "$log" ]] || fail C4 "no wpilog written to $logdir"
  pass C4
  python3 -I tools/elevator/check_sim_log.py "$log" || fail C5 "sim log assertions"
  pass C5
  # C8: no per-loop leaks. Loop time must not grow, and the hub tracker (a single subsystem)
  # must log its state a handful of times, not once per loop.
  python3 -I tools/elevator/check_loop_time.py "$log" || fail C8 "loop time grows"
  dl="$(ls -t logs/FRC_*.wpilog 2>/dev/null | head -1 || true)"
  [[ -n "$dl" && "$dl" -nt "$marker" ]] || fail C8 "no DataLogManager log from this run"
  python3 -I - "$dl" <<'EOF' || fail C8 "hub tracker logs every loop"
import sys
sys.path.insert(0, "tools/elevator")
import wpilog
n = len(wpilog.read(sys.argv[1]).get("/subsystems/HubTracker/isHubActive", ("", []))[1])
print(f"hub tracker state appends: {n}")
sys.exit(0 if n <= 3 else 1)
EOF
  pass C8
  python3 -I tools/elevator/check_can_errors.py "$logdir/sim_console.txt" || fail C9 "CAN device keeps failing"
  pass C9
else
  echo "SKIP C4 C5 (SKIP_E2E=1)"
fi

# ---- C6 -------------------------------------------------------------------------------
python3 -I - <<'EOF' || fail C6 "AdvantageScope layout"
import json, sys
d = json.load(open("src/main/deploy/AdvantageScope-Layout.json"))
cfg = json.load(open("ascope_assets/Robot_GatorboticsElevator/config.json"))
tabs = [t for h in d["hubs"] for t in h["state"]["tabs"]["tabs"] if t["type"] == 3]
if not tabs:
    sys.exit("no 3D field tab")
src = tabs[0]["controller"]["sources"]
want = [("robot", "/RealOutputs/Drive/Odometry/Robot"),
        ("component", "/RealOutputs/Mech/Elevator/ComponentPoses"),
        ("ghost", "/RealOutputs/Drive/Odometry/Robot"),
        ("component", "/RealOutputs/Mech/Elevator/ComponentPosesGoal")]
got = [(s["type"], s["logKey"]) for s in src]
if got != want:
    sys.exit(f"3D sources {got} != {want} (a component attaches to the source before it)")
for s in src:
    if s["type"] in ("robot", "ghost") and s["options"].get("model") != cfg["name"]:
        sys.exit(f"{s['type']} model {s['options'].get('model')} != {cfg['name']}")
# every key the 3D view reads must really be logged, or AdvantageScope silently draws nothing
import glob
sys.path.insert(0, "tools/elevator")
import wpilog
logs = sorted(glob.glob("logs/elevator_e2e/*.wpilog"))
if not logs:
    sys.exit("no E2E log to verify layout keys against (run without SKIP_E2E)")
logged = wpilog.read(logs[-1])
missing = sorted({s["logKey"] for s in src} - set(logged))
if missing:
    sys.exit(f"layout reads keys the robot never logs: {missing}")
EOF
pass C6

# ---- C7 -------------------------------------------------------------------------------
allow='^(RULES\.md|docs/elevator-3d-sim-brief\.md|simgui-ds\.json|build\.gradle|tools/elevator/.*|ascope_assets/Robot_GatorboticsElevator/.*|src/main/deploy/AdvantageScope-Layout\.json|src/main/java/frc/robot/(Constants|Robot|RobotContainer)\.java|src/main/java/frc/robot/subsystems/elevator/.*|src/main/java/frc/robot/commands/mech/ElevatorCommands\.java|src/main/java/frc/robot/util/SimE2E\.java|src/test/java/frc/robot/subsystems/elevator/.*|src/main/java/frc/robot/(commands/mech/(Hood|Intake|Shooting)Commands|subsystems/mech/(IntakeSubsystem|TurretSubsystem)|util/Calculations)\.java)$'
changed="$( { git diff --name-only main...HEAD; git diff --name-only; git ls-files --others --exclude-standard; } | sort -u)"
bad="$(echo "$changed" | grep -v -E "$allow" | grep -v '^$' || true)"
[[ -z "$bad" ]] || fail C7 "files outside plan: $bad"
pass C7

echo "ALL CHECKS PASSED"
