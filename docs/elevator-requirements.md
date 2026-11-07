# Elevator: requirements

What the elevator must do, written before the code (mechanism-checklist.md, section 0).
Each requirement names the test that proves it. If you change a requirement, change its test.

## Shape and parts

- **Shape:** linear (moves along a line). Two-stage cascade: the carriage moves twice as far as
  stage 1.
- **Hardware:** one TalonFX (Kraken X60) and one bottom limit switch on a roboRIO DIO port.
  Wiring is in `config_alecto.properties` as `mech.elevator_*`.
- **Units:** every height is the carriage height in meters above the bottom hard stop.
- **Starting assumption:** the robot can boot with the carriage anywhere. The motor's position
  reads 0 at boot wherever the carriage really is, so the elevator starts **not homed**.

## What the rest of the robot calls

| Method | What it does |
|---|---|
| `setGoal(heightMeters)` | Move to a height and hold it there. |
| `setVoltage(volts)` | Drive by voltage (for testing). Negative is down. |
| `startHoming()` | Drive down slowly until the bottom switch closes. |
| `stop()` | Stop driving the motor. |
| `isHomed()` | True once the bottom switch has been seen. |
| `atGoal()` | True when holding a position, close enough to the goal. |
| `homingFailed()` | True when the last homing gave up. |
| `getPositionMeters()`, `getGoalMeters()` | The current height and the goal. |

Commands (`ElevatorCommands`): `home`, `goTo(height)`, `demoCycle`.

## Requirements

**Homing**

| # | Requirement | Test |
|---|---|---|
| R1 | Before homing, a goal never moves the motor. | `refusesToMoveBeforeHoming` |
| R2 | When the bottom switch closes, the position is set to 0 and the elevator is homed. This happens once per press, not every loop. | `switchPressHomesOnce` |
| R3 | Homing drives down at -1.5 V until the switch closes, then holds the bottom. | `homingDrivesDownUntilSwitch` |
| R4 | If the switch has not closed after 200 loops (4 s), homing gives up: the motor stops and the "homing failed" alert turns on. | `homingGivesUpAfterTimeout` |

**Moving**

| # | Requirement | Test |
|---|---|---|
| R5 | Goals are limited to 0 to 1.20 m. | `goalIsClampedToTravel` |
| R6 | A goal above the bottom is sent to the motor's onboard Motion Magic. | `goalIsSentToMotor` |
| R7 | Going to the bottom: Motion Magic to 0.03 m, then a gentle -0.5 V push, then no power at all once the switch closes (no holding current on the hard stop). | `landsGentlyAtTheBottom` |
| R8 | Driving by voltage passes the voltage through, but stops when driving down onto a pressed switch. | `voltageStopsAtBottomSwitch` |
| R9 | When the robot is disabled, the elevator forgets its goal and stays stopped after re-enabling. | `disablingForgetsTheGoal` |
| R10 | `atGoal()` is true only when homed, holding a position, and within 0.01 m. | `atGoalNeedsHomedAndClose` |

**Problems made easy to see**

| # | Requirement | Test |
|---|---|---|
| R11 | Motor disconnected: alert on while it is disconnected. | `disconnectedMotorTurnsOnAlert` |
| R12 | Told to move but not at the goal after 100 loops (2 s): "not arrived" alert on. It turns off on arrival or a new goal. | `notArrivedAlertAfterTimeout` |
| R13 | Gains changed on the dashboard (`/Tuning/Elevator/k*`) are sent to the motor once, only when they change. | `gainsSentOnlyWhenChanged` |

**Logging and 3D view**

| # | Requirement | Test |
|---|---|---|
| R14 | Hardware readings are logged under `Mech/Elevator/IO`. Goal, mode, homed and at-goal are logged under `Mech/Elevator/...`. | `tools/elevator/check.sh` C5 |
| R15 | `Mech/Elevator/ComponentPoses` holds one pose per moving part of the 3D model: stage 1 at half the carriage height, the carriage at its height. `ComponentPosesGoal` is the same for the goal. | `componentPosesFollowCascade` |

## Simulator (physics) requirements

These cover `ElevatorIOSimPhysics`, the simulator that runs the real motor code with physics.

| # | Requirement | Test |
|---|---|---|
| S1 | kG, kV and kA are within 10% of what physics says for the masses and gearing. | `ElevatorPhysicsTest` |
| S2 | The 3D model matches the constants (sizes, travel). | `ElevatorModelTest`, `check.sh` C3 |
| S3 | In the full robot sim, the elevator homes, reaches every setpoint in time and stays in its travel. | `check.sh` C4, C5 |
