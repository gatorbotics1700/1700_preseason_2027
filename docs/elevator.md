# Elevator prototype

A working two-stage elevator with a 3D view in AdvantageScope. It proves out a faster way to build
mechanisms: test them in the simulator and preview them in 3D before the real robot exists. It
follows `docs/3d-sim-workflow.md` step by step, so use it as the example for your mechanism.

No robot has an elevator yet: `ElevatorConstants.ENABLED_ON_REAL_ROBOT = false`.

**References:** 1678's 2025 code, https://github.com/frc1678/C2025-Public. There's no CAD yet, so
`view3d/ElevatorModel.java` generates a stand-in 3D model (boxes) from `ElevatorConstants`, and
`view3d/ElevatorModelTest.java` rebuilds it whenever a constant changes. The 3D view always
matches the code. When real CAD arrives, replace the generated files and keep the size tests.

## Files

| What | Where |
|---|---|
| Decisions: homing, moving, landing, alerts | `subsystems/mech/ElevatorSubsystem.java` |
| Hardware: real motor, physics sim, test stand-in, do-nothing | `ElevatorIO`, `ElevatorIOTalonFX`, `ElevatorIOSimPhysics`, `ElevatorIOSim`, `new ElevatorIO() {}` |
| Commands: home, go to a height, demo auto | `commands/mech/ElevatorCommands.java` |
| Numbers | `ElevatorConstants` in `Constants.java`; wiring `mech.elevator_*` in `config_alecto.properties` |
| Unit tests | `subsystems/mech/ElevatorSubsystemTest.java`, `ElevatorPhysicsTest.java` |
| 3D model and its checks | `view3d/ElevatorModel.java`, `ElevatorModelTest.java`, `AdvantageScopeLayoutTest.java` |
| Log checks: the shared `PositionChecks` set up for the elevator, plus its own | `diagnostics/ElevatorChecks.java` |
| 3D files and layout | `ascope_assets/Robot_GatorboticsElevator/`, `src/main/deploy/AdvantageScope-Layout-Elevator.json` |

## How it works

- Moves to a height with the motor's onboard Motion Magic, in meters.
- Homes on the bottom limit switch and won't move until homed. Homing gives up after 4 s.
- Going to the bottom, it slows to a gentle push and then turns the motor off, instead of holding
  current against the hard stop.
- Alerts: motor disconnected, homing failed, told to move but not there after 2 s.

## Testing it

1. **Unit tests:** VS Code task **Gatorbotics: Run code tests**.
2. **Smoke test:** task **Gatorbotics: Smoke test sim**, type `Elevator Demo`. It homes, visits
   every height, and checks the log. Ends with `0 errors`.
3. **Drive it:** `WPILib: Simulate Robot Code` > Sim GUI > Teleoperated, click into the Sim GUI.
   No key setup needed (Keyboard 2 on joystick 2):

   | Key | Elevator |
   |---|---|
   | Up arrow | high (1.15 m) |
   | Right arrow | mid (0.80 m) |
   | Left arrow | low (0.40 m) |
   | Down arrow | stow (0 m) |
   | Home (Fn+Left on a Mac laptop) | home |

   On a real robot: the codriver's D-pad picks the height and Back homes.
4. **Watch it in 3D:** in AdvantageScope, **Use Custom Assets Folder...** > `ascope_assets`, then
   **File > Import Layout...** > `src/main/deploy/AdvantageScope-Layout-Elevator.json`. Connect
   live with **File > Connect to Simulator > NetworkTables 4 (AdvantageKit)**, or open the log in
   `logs/sim_smoke/` after a smoke test.

## Placeholders

The CAN ID, DIO port, masses, gearing, travel and sizes are guesses, marked `TUNED, NOT DERIVED`.
The build team needs to measure them on a real elevator. kG, kV and kA are worked out from the
motor model and masses, and `ElevatorPhysicsTest` checks them.

## What we learned

- Size the current limit for the hardest stop: slowing down while moving down. At its limit, the
  motor can't follow the profile, and it looks like a tuning problem.
- Check that the carriage and stages clear each other through the whole travel, not just at rest.
- Step the physics on its own 5 ms thread. At 20 ms the simulated elevator overshoots far more
  than a real one would.
