# Simulate a mechanism in 3D, and check it automatically

This guide adds a physics simulator, a 3D view and automatic log checks to a mechanism. The
**elevator** is the worked example: every step points at its files, so open them side by side.

Do this after CLAUDE.md's Steps 0 to 7: the mechanism already has an IO layer (`XIO`,
`XIOTalonFX`, the stand-in `XIOSim`, the do-nothing `new XIO() {}`) and subsystem tests. This
guide is Step 8 (simulator physics) and more.

## What you get

| Layer | What it proves | How to run it in VS Code |
|---|---|---|
| Unit tests | The decisions, the math, and that the 3D model matches the constants | Task `Gatorbotics: Run code tests` |
| Physics simulator | The mechanism really moves, with the real motor code | `WPILib: Simulate Robot Code` |
| 3D view | You can watch it move, live or in a log | AdvantageScope (below) |
| Sim smoke | The whole robot runs an auto in the simulator, then the log checks pass | Task `Gatorbotics: Smoke test sim` |
| Log diagnostics | Finds problems in any log: sim, practice or a match | Task `Gatorbotics: Diagnose log file` |

**Running a task:** press Cmd+Shift+P (Ctrl+Shift+P on Windows), type `Run Task`, pick
**Tasks: Run Task**, then pick one of the `Gatorbotics:` tasks. The output shows in the Terminal panel.
The tasks are in `.vscode/tasks.json`:

| Task | What it does |
|---|---|
| `Gatorbotics: Run code tests` | Builds the code and runs every unit test (about 10 seconds) |
| `Gatorbotics: Smoke test sim` | Asks which auto to run (press Enter for Elevator Demo), runs it in the simulator, then checks the log (about 10 seconds) |
| `Gatorbotics: Diagnose log file` | Asks for a log file or folder (press Enter for the newest sim smoke log), then prints the report |

**Before asking for review**, run `Run code tests`, then `Smoke test sim`. Both must end with
`BUILD SUCCESSFUL`.

Prefer a terminal? Each task is one command: `./gradlew build`, `./gradlew simSmoke
-PsimSmokeAuto="<name>"`, `./gradlew diagnoseLog -Plog=<file>`. On Windows, type `gradlew.bat`
instead of `./gradlew`. On a Mac, if it says Java is missing, put `JAVA_HOME=~/wpilib/2026/jdk `
in front.

## The files, using the elevator as the example

| What | Elevator file |
|---|---|
| Stand-in hardware for unit tests | `subsystems/mech/ElevatorIOSim.java` |
| Physics simulator | `subsystems/mech/ElevatorIOSimPhysics.java` |
| Poses for the 3D view | `ElevatorSubsystem.componentPoses()` |
| 3D model, built from the constants | `src/test/java/frc/robot/view3d/ElevatorModel.java` |
| 3D model checks | `src/test/java/frc/robot/view3d/ElevatorModelTest.java` |
| Layout checks | `src/test/java/frc/robot/view3d/AdvantageScopeLayoutTest.java` |
| Demo auto for sim smoke | `ElevatorCommands.demoCycle()`, added to the auto chooser as "Elevator Demo" |
| Log checks | `src/test/java/frc/robot/diagnostics/ElevatorChecks.java` |

## Step A: a physics simulator

The stand-in `XIOSim` only remembers what it was told, so nothing moves. The physics simulator
makes the mechanism move like the real one.

1. Make `XIOSimPhysics extends XIOTalonFX`. It runs your **real** motor code against Phoenix's
   simulated TalonFX, so the same onboard control (Motion Magic, current limits) runs in the
   simulator and on the robot.
2. Pick the WPILib model that fits the shape: `ElevatorSim` (elevator, climber),
   `SingleJointedArmSim` (arm, hood), `FlywheelSim` or `DCMotorSim` (rollers, turret).
3. Step the model on its own 5 ms thread (a `Notifier`), not once per 20 ms loop: the TalonFX
   controls at 1 kHz and needs fresh positions.
4. Drive the simulated sensors from the simulated position (the elevator presses its bottom
   switch when the carriage is at the bottom), so homing runs by itself.
5. In `RobotContainer`, use `XIOSimPhysics` in the `SIM` case.
6. The model's numbers (mass, gearing, travel) must come from the **build team**, not from
   copying the code's constants. Otherwise the simulator only agrees with the code. Until then,
   mark them `// TUNED, NOT DERIVED (date)`.

Test it: `ElevatorPhysicsTest` checks that kG, kV and kA match the physics within 10%.

## Step B: log a pose for each moving part

AdvantageScope moves each part of the 3D model by a `Pose3d` (where it is and how it is turned).

1. Add a `static Pose3d[] componentPoses(...)` to the subsystem: one pose per moving part, in a
   fixed order. Each pose is how far that part moved from its "zero" position.
2. In `periodic()`, log `Mech/<Name>/ComponentPoses`, and `Mech/<Name>/ComponentPosesGoal` for a
   see-through "ghost" at the goal.
3. Test the math (see `componentPosesFollowCascadeRigging`).

## Step C: a 3D model

AdvantageScope needs a folder in `ascope_assets/` with a `config.json` and `.glb` model files:
`model.glb` (the robot, moves with the robot) and `model_0.glb`, `model_1.glb`, ... (one per
moving part, in the same order as ComponentPoses).

- **No CAD yet?** Build a stand-in from boxes, sized by your constants: copy `ElevatorModel.java`
  and `ElevatorModelTest.java`. The test rebuilds the files whenever a constant changes, and fails
  once so you notice: run the tests again and commit the new files.
- **CAD from the build team?** Export each part from Onshape as glTF (`.glb`), positioned where it
  sits at "zero". Keep the size and clearance tests from `ElevatorModelTest`, so the code's
  constants and the CAD can't drift apart.

## Step D: an AdvantageScope layout

Give the mechanism its own layout file, and leave the team layout
(`src/main/deploy/AdvantageScope-Layout.json`) alone. The elevator's is
`src/main/deploy/AdvantageScope-Layout-Elevator.json`: one 3D Field tab with the robot model, then a
"component" source reading `/RealOutputs/Mech/<Name>/ComponentPoses`. A component attaches to the
robot listed just before it. Check it with a test like `AdvantageScopeLayoutTest`.

## Step E: a demo auto and sim smoke

1. Write a command that exercises the mechanism (`ElevatorCommands.demoCycle()`: home, then
   visit every height) and add it to the auto chooser in `RobotContainer`.
2. Run the task `Gatorbotics: Smoke test sim` and type its name. The robot starts in the simulator with
   no window, picks that auto, enables autonomous, waits for the auto to end, then runs the log
   checks. The log and console are saved in `logs/sim_smoke/`.

## Step F: log checks

A log check looks for one problem in a log. The same checks run on sim smoke logs and on match
logs, so a check you write today keeps finding that problem at competitions.

1. Make `src/test/java/frc/robot/diagnostics/<Name>Checks.java`. Copy the smallest class in
   `ElevatorChecks.java` (`StaysInVoltageAndCurrentLimits`), and add yours to `all()`.
2. List the keys it reads in `requiredKeys()`. A log without them skips the check instead of
   failing it. Add `SimSmokeKeys.AUTO` for a check that only makes sense in the simulator.
3. Add `checks.addAll(<Name>Checks.all());` to `LogDiagnostics.allChecks()`.
4. Test it with a small fake log (`TestLog`), like `ElevatorChecksTest`: one log that should
   pass, one that should fail. Break the check on purpose once and watch the test fail.

Good things to check, from the checklist's three questions: does every device stay connected?
does it reach its goal in time without overshooting? does it stay inside its travel, voltage and
current limits? does the 3D pose agree with the position?

## Watching it in AdvantageScope

Once:

1. **AdvantageScope > Use Custom Assets Folder...** and pick the `ascope_assets` folder.
2. **File > Import Layout...** and pick the mechanism's layout, for example
   `src/main/deploy/AdvantageScope-Layout-Elevator.json`.

Live: start `WPILib: Simulate Robot Code` (Sim GUI), then in AdvantageScope **File > Connect to
Simulator > NetworkTables 4 (AdvantageKit)**. Plain "NetworkTables 4" puts `/AdvantageKit` in front
of every key, so the 3D view finds nothing.

From a log: run the task `Gatorbotics: Smoke test sim`, then **File > Open Log(s)...** and pick the `.wpilog` in
`logs/sim_smoke/`. Drag the timeline to scrub.

Quit AdvantageScope with Cmd+Q (Mac). Closing the window resets the layout.

## Driving it by hand in the Sim GUI

Start `WPILib: Simulate Robot Code` (Sim GUI), click Disabled, then Teleoperated. Click into the
Sim GUI window so it gets your key presses.

**Elevator, no setup needed:** the team's Sim GUI setup already puts Keyboard 2 on joystick 2, and
in the simulator the elevator listens to it:

| Key | Elevator |
|---|---|
| Up arrow | high (1.15 m) |
| Right arrow | mid (0.80 m) |
| Left arrow | low (0.40 m) |
| Down arrow | stow (0 m) |
| Home (Fn+Left arrow on a Mac laptop) | home |

If nothing happens, check that the Sim GUI's Joysticks panel shows Keyboard 2 on Joystick[2].

**Your own mechanism:** do the same. Bind it, in the simulator only, to a keyboard that is already
set up, on a joystick port no robot code uses (see `configureElevatorSimKeys()` in
`RobotContainer`). Then nobody has to remap keys. On the real robot, the codriver's D-pad and Back
button drive the elevator.

If you do remap keys (for example the codriver's D-pad on a Mac laptop, which has no numpad: map
the POV to keys in **DS > Keyboard 1 Settings**; the Mac simulator reads Back as button 11), don't
commit `simgui-ds.json`: it holds your own key setup.

## Checking a match log

1. Download the logs from the robot: AdvantageScope **File > Download Logs...**, or copy them
   from the USB stick.
2. Put the log in the project folder (for example in `logs/`), right-click it in the VS Code
   Explorer, and pick **Copy Path**.
3. Run the task `Gatorbotics: Diagnose log file` and paste the path (a folder uses its newest log).
4. Read the report: `FAIL` lines are problems, with the time they happened. Open the same log in
   AdvantageScope and jump to that time.

## Lessons

- A part on a moving stage must clear every other stage through the whole travel, not just at rest.
- Work out kG, kV and kA from the motor model and the masses, and pin them with a test.
- Before tuning gains, log the motion profile's target next to the position. If the motor sits at
  its current limit, no gain can fix it.
- AdvantageKit logs a value only when it changes. A check must ask "what was it at this time?".
- Test like a match: set an alliance.
- Check every layout key against a real log. A key nobody logs makes AdvantageScope draw nothing.
- When a check fails, first make sure it measured the right thing.

## What the simulator cannot show

Jams, binding, chain stretch, real current under load, and anything the build team hasn't
measured yet. The simulator proves the code; only the robot proves the mechanism.
