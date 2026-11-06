# Elevator task - attempt log

Every attempt per unit, including rejected ones. Format: `UNIT | attempt | change | check | verdict | note`.

| Unit | # | Change | Check | Verdict | Note |
|---|---|---|---|---|---|
| U1 constants | 1 | Add ElevatorConstants | compileJava exit 0 | PASS | hand geometry check found carriage overran stage 1 by 2cm at max; STAGE_1_HEIGHT 0.85->0.88, invariant to be enforced by ElevatorKinematicsTest |
| U0 tests-enable | 1 | Uncomment test{} + configureTestTasks in build.gradle | gradlew test exit 0 | PASS (weak) | existing test files are fully commented out, so 0 tests ran; real gate is C2 |
| U2 model | 1 | generate_model.py, shelf forward | --check exit 0, winding vs normals, bbox vs constants | REJECT | shelf at x up to 0.238 sweeps through fixed/stage-1 top crossbars (x 0.024..0.126) during travel |
| U2 model | 2 | shelf moved to carriage back face | same + swept-volume clearance (shelf x < rails min x) | PASS | |
| U1 constants | 2 | kG/kV/kA replaced with values derived from the Kraken X60 model (0.60->0.80, 2.65->2.55, 0.05->0.07) | first-principles calc | PASS | guessed kG was 25% low |
| U3 IO layer | 1 | ElevatorIO + ElevatorIOTalonFX | compileJava exit 0 | PASS | |
| U4 sim IO | 1 | ElevatorIOSim (Phoenix TalonFX sim + WPILib ElevatorSim + DIOSim) | compileJava exit 0 | PASS | |
| U5 subsystem | 1 | ElevatorSubsystem + ElevatorCommands | compileJava exit 0 | PASS | |
| U7 unit tests | 1 | Kinematics/Physics/Model tests (23) | check_junit.py exit 0 | PASS | |
| U7 unit tests | 1b | mutation: STAGE_1 0.85 + KG 0.60 | tests must fail | PASS | 4 failures caught as expected; constants restored |
| U5 subsystem | 2 | Go neutral on the bottom hard stop when goal is stow (was holding ~31 A forever) | new unit test restsOnHardStopInsteadOfHoldingAtBottom + final current in E2E log 0.0 A | PASS | |
| U6 wiring | 1 | RobotContainer per-mode IO, auto option, codriver D-pad/Back, SimE2E hook in Robot | compileJava exit 0 | PASS | |
| U8 e2e | 1 | run_e2e_sim.sh + wpilog.py + check_sim_log.py, first run | check.sh C5 | REJECT | L2 L4 L5 L6 failed |
| U8 e2e | 2 | checker fixes only: seed goal sequence (AK logs changes only), skip pre-reset homing cycle in L5, L6 compares only when stationary (20 ms sensor latency is not offset error), L3 must stay in band | same log re-checked | REJECT (expected) | only L4 left = real defect: 12.2 cm undershoot on 1.15->0.80 |
| U8 e2e | 3 | ElevatorIOSim plant on a 5 ms Notifier thread (TalonFX loops at 1 kHz on 20 ms stale positions) | check.sh C5 | REJECT, REVERTED | L4 12.2->3.9 cm, but new L1 (carriage slides onto switch during boot) and L8 (65.2 A, one sample, hard-stop impact). Saved as rejected/u8-attempt3-5ms-sim-thread.patch. 3 attempts used: unit stopped |
| U8a sim fidelity | 1 | Re-apply 5 ms sim thread patch (approved revised plan) | E2E L4 12.2->3.9 cm | PASS | |
| U8b check L1 | 1 | L1 = switch closes then Homed, before any raised goal (boot homing is physical) | re-check attempt-3 log: only L4, L8 left | PASS | |
| U10 soft landing | 1 | Bottom goal: Motion Magic to 3 cm, then -0.5 V onto stop, neutral on switch | E2E L8 65.2->58.2 A + unit test landsGentlyWhenGoingToTheBottom | PASS | |
| U11 tuning | 0 | Log Motion Magic ClosedLoopReference as input (diagnostic, no behavior change) | trace | INFO | tracking fine until decel; stator pinned at 60 A limit while decelerating downward -> not a gain problem |
| U11 tuning | 1 | Stator limit 60->80 A + test: worst-case down-decel current <= 60% of limit | check.sh C1-C5 | PASS | 1.15->0.80 settles 0.42 s, peak 70.7 A |
| U9 layout | 1 | 3D Field: robot + component(ComponentPoses), ghost + component(ComponentPosesGoal), model = generated robot; C6 checks order and model name | check.sh C6 | PASS | schema confirmed from AdvantageScope v26 bundle (component childOf robot; ghost parentKey robot) |
| U12 hub tracker | 0 | Repro: E2E with alliance Red1 (FMS-like) + 10 s hold | DataLog /subsystems/HubTracker/isHubActive appends | REPRO | 722 appends + reportWarning per loop in 14.4 s; loop time flat even at 64 s, so CPU is not the symptom |
| U12 hub tracker | 1 | RobotContainer builds HubTrackerSubsystem once as a field (was new instance every periodic) | check.sh C8 (appends <= 3, loop time flat) | PASS | 722 -> 1 |
| U13 turret CAN | 0 | Repro: check_can_errors.py on E2E console | repeated (device, signal) errors | REPRO | talon fx 100: 11 signals repeating (CAN IDs are 0-62; ID 100 was a hack to keep the turret still) |
| U13 turret CAN | 1 | TurretConstants.ENABLED=false gates config/control/reads; ID restored to 15 | check.sh C9 | PASS | 41 CAN errors (11 repeating) -> 1 one-time boot error |
| tooling | 1 | run_e2e_sim.sh kills watchdog's sleep child (held stdout open 240 s) | check.sh wall time | PASS | 4+ min -> 11.4 s |
| U9 layout | 2 | Robot/ghost logKey /RealOutputs/Odometry/Robot -> /RealOutputs/Drive/Odometry/Robot (old key never logged: nothing rendered); C6 now verifies every layout key exists in the E2E log | C6 with stale key restored must fail | PASS | found by the user: model selectable but nothing in 3D view |
