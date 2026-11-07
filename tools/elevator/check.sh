#!/usr/bin/env bash
# Pass condition for the elevator + 3D sim task. Exit 0 == task passes. Nothing else counts.
#
#   C1  ./gradlew build exits 0 (compile + spotlessCheck + unit tests, tests actually enabled)
#   C2  JUnit results: required elevator test classes ran, 0 failures / errors / skips
#   C3  Generated CAD model (ascope_assets/) is byte-identical to a fresh regeneration
#       from ElevatorConstants (CAD and code share one source of truth)
#   C4  ./gradlew simSmoke: the robot runs the Elevator Demo auto in the simulator and the Java
#       log diagnostics find no errors
#   C6  AdvantageScope layout JSON parses and references the elevator component poses
#   C7  Every changed file (vs main) is inside the plan allowlist
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
  frc.robot.subsystems.mech.ElevatorSubsystemTest \
  frc.robot.subsystems.mech.ElevatorKinematicsTest \
  frc.robot.subsystems.mech.ElevatorPhysicsTest \
  frc.robot.subsystems.mech.ElevatorModelTest \
  frc.robot.diagnostics.LogFileTest \
  frc.robot.diagnostics.ElevatorChecksTest \
  frc.robot.diagnostics.RobotChecksTest \
  || fail C2 "junit results"
pass C2

# ---- C3 -------------------------------------------------------------------------------
python3 -I tools/elevator/generate_model.py --check || fail C3 "model out of sync with constants"
pass C3

# ---- C4 -------------------------------------------------------------------------------
# Sim smoke: the whole robot runs the Elevator Demo auto in the simulator, then the Java log
# diagnostics check the log (homing, moves, travel, limits, devices, CAN, start pose).
if [[ "${SKIP_E2E:-0}" != "1" ]]; then
  smokelog="$(mktemp)"
  code=0
  ./gradlew simSmoke --offline -q >"$smokelog" 2>&1 || code=$?
  grep -E "SimSmoke\]|^(FAIL|WARN)|ERROR|errors," "$smokelog" || true
  [[ "$code" == "0" ]] || { tail -20 "$smokelog" >&2; fail C4 "sim smoke"; }
  pass C4
else
  echo "SKIP C4 (SKIP_E2E=1)"
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
logs = sorted(glob.glob("logs/sim_smoke/*.wpilog"))
if not logs:
    sys.exit("no E2E log to verify layout keys against (run without SKIP_E2E)")
logged = wpilog.read(logs[-1])
missing = sorted({s["logKey"] for s in src} - set(logged))
if missing:
    sys.exit(f"layout reads keys the robot never logs: {missing}")
EOF
pass C6

# ---- C7 -------------------------------------------------------------------------------
allow='^(RULES\.md|docs/(elevator-3d-sim-brief|elevator-requirements|3d-sim-workflow)\.md|simgui-ds\.json|build\.gradle|tools/elevator/.*|ascope_assets/Robot_GatorboticsElevator/.*|src/main/deploy/AdvantageScope-Layout\.json|src/main/deploy/configFiles/config_alecto\.properties|src/main/java/frc/robot/(Constants|Robot|RobotContainer)\.java|src/main/java/frc/robot/subsystems/mech/Elevator.*|src/main/java/frc/robot/commands/mech/ElevatorCommands\.java|src/main/java/frc/robot/util/SimSmoke\.java|src/test/java/frc/robot/subsystems/mech/Elevator.*|src/test/java/frc/robot/(diagnostics|view3d)/.*|src/main/java/frc/robot/(commands/mech/(Hood|Intake|Shooting)Commands|subsystems/mech/(IntakeSubsystem|TurretSubsystem)|util/Calculations)\.java)$'
# compare against the remote main when there is one: a local main branch can be stale
base="$(git rev-parse --verify -q origin/main >/dev/null && echo origin/main || echo main)"
# merge base vs the working tree: committed and uncommitted changes, as the branch would merge
changed="$( { git diff --name-only "$(git merge-base "$base" HEAD)"; git ls-files --others --exclude-standard; } | sort -u)"
bad="$(echo "$changed" | grep -v -E "$allow" | grep -v '^$' || true)"
[[ -z "$bad" ]] || fail C7 "files outside plan: $bad"
pass C7

echo "ALL CHECKS PASSED"
