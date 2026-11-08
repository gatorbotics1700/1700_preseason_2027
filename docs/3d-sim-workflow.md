# Simulate a mechanism in 3D, and check it automatically

How to add a physics simulator, a 3D view and log checks to a mechanism, so you can test it and
see it move before the real robot exists. Do this after CLAUDE.md's Steps 0 to 7 (the mechanism
already has its IO layer and tests). Worked example: the elevator on branch
`mentors/szymon/elevator-prototype` (see `docs/elevator.md` there).

## The tools

Run them in VS Code: Cmd+Shift+P (Ctrl+Shift+P on Windows) > **Tasks: Run Task** > pick one.

| Task | What it's for | Terminal |
|---|---|---|
| **Gatorbotics: Run code tests** | Builds and runs every unit test | `./gradlew build` |
| **Gatorbotics: Smoke test sim** | Runs one auto on the whole robot in the simulator, then checks its log | `./gradlew simSmoke -PsimSmokeAuto="<name>"` |
| **Gatorbotics: Diagnose log file** | Finds problems in any log: sim, practice or a match | `./gradlew diagnoseLog -Plog=<file>` |

Before asking for review, run code tests, then smoke test sim. Both must end with
`BUILD SUCCESSFUL`. (Terminal on Windows: `gradlew.bat`. On a Mac, if Java is missing, put
`JAVA_HOME=~/wpilib/2026/jdk ` in front.)

## Steps

1. **Physics simulator.** Make `XIOSimPhysics extends XIOTalonFX`, so the real motor code runs
   against Phoenix's simulated motor. Use the WPILib model that fits: `ElevatorSim`,
   `SingleJointedArmSim`, `FlywheelSim` or `DCMotorSim`. Step it on a 5 ms `Notifier`, drive the
   simulated sensors from it, and use it in `RobotContainer`'s `SIM` case. Its numbers come from
   the build team; until then, mark them `TUNED, NOT DERIVED`.
2. **Poses.** Log one `Pose3d` per moving part as `Mech/<Name>/ComponentPoses` (and
   `ComponentPosesGoal` for a see-through ghost at the goal). Test the math.
3. **3D model.** A folder in `ascope_assets/` with `config.json`, `model.glb` (the robot) and
   `model_0.glb`, `model_1.glb`... (one per pose). No CAD yet? Build boxes from your constants with
   `view3d/Glb`, in a test that rebuilds the files when a constant changes. CAD later? Export
   `.glb` files from Onshape and keep the size tests.
4. **Layout.** Give the mechanism its own AdvantageScope layout file and leave the team layout
   alone. In its 3D tab, list the robot, then a "component" reading your ComponentPoses. Add a
   `View3dChecks.LayoutKeysAreLogged` check for it.
5. **Demo auto.** A command that exercises the mechanism, added to the auto chooser. Run it with
   Smoke test sim.
6. **Log checks.** Make `diagnostics/<Name>Checks.java` (copy a class from `RobotChecks`), list
   the keys each check needs (missing keys skip it), add it to `LogDiagnostics.allChecks()`, and
   test it with a small fake log (`TestLog`).

## Watching it in AdvantageScope

1. Once: **AdvantageScope > Use Custom Assets Folder...** > `ascope_assets`, then
   **File > Import Layout...** > the mechanism's layout.
2. Live: start `WPILib: Simulate Robot Code`, then **File > Connect to Simulator > NetworkTables 4
   (AdvantageKit)**. Plain "NetworkTables 4" shows nothing in the 3D view.
3. From a log: run Smoke test sim, then **File > Open Log(s)...** > `logs/sim_smoke/`.

To drive by hand in the Sim GUI, bind the mechanism (in the simulator only) to a keyboard the Sim
GUI already has, on a joystick port no robot code uses. Then nobody needs to remap keys. Don't
commit `simgui-ds.json`.

## Checking a match log

Download the logs (AdvantageScope **File > Download Logs...**), put the file in `logs/`,
right-click it > **Copy Path**, then run **Gatorbotics: Diagnose log file** and paste it. `FAIL`
lines are problems, with the time they happened.

## Lessons

- Work out kG, kV and kA from the motor model and masses, and pin them with a test.
- Before tuning gains, log the profile's target next to the position. If the motor sits at its
  current limit, no gain can fix it.
- AdvantageKit logs a value only when it changes. A check must ask "what was it at this time?".
- Check every layout key against a real log. A key nobody logs makes AdvantageScope draw nothing.

The simulator can't show jams, binding, chain stretch or real current under load. It proves the
code; only the robot proves the mechanism.
