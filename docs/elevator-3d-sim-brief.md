# Elevator + 3D sim: session brief

Branch `mentors/szymon/research`, Nov 5-7 2026. Local commits only, nothing pushed.
Latest `main` (CLAUDE.md, mechanism-checklist.md, subsystems/template/) is merged in.

## Goal (achieved)

A working elevator mechanism in this code base, with a matching 3D model, that you can watch
move in AdvantageScope's 3D view: live from the simulator, and in log playback with a scrubbable
timeline.

**Why:** prove the workflow end to end on one mechanism, then turn it into the team's standard
tooling and patterns so every future mechanism (hood, turret, climber...) gets sim, 3D view,
logging and checks without re-inventing them.

## What exists now

| Piece | Where |
|---|---|
| Subsystem, AdvantageKit IO layer (real / sim / do-nothing) | `subsystems/elevator/` |
| Commands (home, goTo, demoCycle), "Elevator Demo" auto, codriver D-pad + Back | `commands/mech/ElevatorCommands.java`, `RobotContainer` |
| All numbers (hardware, gearing, gains, geometry) | `ElevatorConstants` in `Constants.java` |
| 3D model, generated from `ElevatorConstants` (no real CAD yet) | `ascope_assets/Robot_GatorboticsElevator/`, `tools/elevator/generate_model.py` |
| AdvantageScope layout (robot + live components + goal ghost) | `src/main/deploy/AdvantageScope-Layout.json` |
| Pass condition (9 checks, about 11 s; must exit 0 after every change) | `tools/elevator/check.sh` |
| Headless E2E sim + log checker | `tools/elevator/run_e2e_sim.sh`, `check_sim_log.py`, `wpilog.py`, `util/SimE2E.java` |
| Unit tests (26 elevator + 3 from main's template): kinematics, safety logic, physics, model vs constants | `src/test/.../subsystems/elevator/` |
| Attempt log and learned rules | `tools/elevator/ATTEMPTS.md`, `RULES.md` |

**How it works:** the sim runs the real `ElevatorIOTalonFX` code against Phoenix's simulated
TalonFX (same onboard Motion Magic as the robot), with WPILib `ElevatorSim` physics stepped on a
5 ms thread. The subsystem logs `Pose3d[]` component poses each loop. AdvantageScope draws the
robot model at the drive pose and moves stage 1 and the carriage with those poses.

**Also fixed on the way:** `HubTrackerSubsystem` re-created every loop (722 warnings in 14 s);
turret disabled with fake CAN ID 100 (CAN errors every loop); stale robot pose key in the
AdvantageScope layout (nothing rendered); sim robot started in the field corner.

## How to use it (people)

1. AdvantageScope: **AdvantageScope > Use Custom Assets Folder** > pick `ascope_assets`.
2. **File > Import Layout...** > `src/main/deploy/AdvantageScope-Layout.json`.
3. Live: start the sim, then **File > Connect to Simulator > NetworkTables 4 (AdvantageKit)**.
   Plain "NetworkTables 4" shows keys with an `/AdvantageKit` prefix, so nothing matches.
4. Playback: `ELEVATOR_E2E_LOG_DIR=logs/demo ./gradlew simulateJava`, then **File > Open Log(s)...**.
5. Drive it: Sim GUI, Keyboard 0 in joystick slot 1, Teleoperated, arrow keys = heights, H = home
   (needs the uncommitted `simgui-ds.json`, see open items). Or the "Elevator Demo" auto.
6. Quit AdvantageScope with Cmd+Q. Closing the window resets the layout.

## What we learned (keep these)

- Generate the model from the same constants the code uses, and test the model against them.
  Then the 3D view can never drift from the code. When real CAD arrives, derive the constants
  from CAD and keep the test.
- Check behavior in the real robot loop (headless sim + log assertions), not just unit tests.
  Every real bug here (overshoot, current limiting, missing pose key, leaks) was found that way.
- Before tuning gains, log the profile reference next to position. Our overshoot was the 60 A
  stator limit, not the gains.
- Verify every dashboard/layout key against a real log. Never copy keys from an old layout.
- Full list: `RULES.md`. Every attempt, including rejected ones: `tools/elevator/ATTEMPTS.md`.

## Next session

**Start here.** Run `tools/elevator/check.sh` first: it must print `ALL CHECKS PASSED`. Then read
`CLAUDE.md`, `mechanism-checklist.md` and `src/main/java/frc/robot/subsystems/template/README.md`.
They are the team's rules and patterns, and this branch must follow them.

**1. Merge latest `main`. DONE (Nov 7).** Clean merge, no conflicts. `main`'s
`TemplateSubsystemTest` had never run (tests are off on `main`); on this branch it runs and passes.
`check.sh` C7 now diffs against `origin/main`, because a local `main` can be stale.

**2. Bring the elevator in line with `main`'s CLAUDE.md.** Known gaps:

- Tests: we turned on `test {}` in `build.gradle`. CLAUDE.md says ask a mentor first (the intake
  branch uses `forkEvery = 1`). Decide with the team.
- `static` `LoggedNetworkNumber` gains in `ElevatorSubsystem`: CLAUDE.md wants normal fields.
- Wiring (CAN 40, DIO 5) is in `ElevatorConstants`: move to `config_alecto.properties` as
  `mech.elevator_*`, read via `RobotConfigLoader`.
- Homing timeout uses `withTimeout` (wall clock): count loops instead.
- Code style: switch arrows, records in tests, larger lambdas. Simplify to CLAUDE.md's level.
- Turret: we used an `ENABLED` flag. CLAUDE.md asks for a do-nothing IO. Redo it once the turret
  gets its IO layer.
- Fold `RULES.md` into CLAUDE.md's lessons and update its "3D: not done on this team yet" note.

**3. Turn the elevator into a reusable framework.** Proposed shape (confirm with mentors):

- Shared mechanism patterns by shape, as `mechanism-checklist.md` already plans: speed (roller),
  position (pivot, hood, turret), linear (elevator, climber). The elevator becomes the linear one,
  `subsystems/template/` the starting point.
- One standard: every mechanism logs `Mech/<Name>/ComponentPoses`, has a do-nothing IO, and
  picks real / sim / do-nothing IO by mode in `RobotContainer`.
- Generalize `tools/elevator/` into `tools/mechanism/`: the model generator driven by a small
  per-mechanism geometry spec (or a real CAD `.glb` dropped in), a robot config with components
  for every mechanism, one layout, and a `check.sh` that runs the same checks for any mechanism.
- Real CAD path: Onshape > STEP > `.glb` (AdvantageScope docs: Custom Assets > glTF convert).
  Need the build team's export plus measured masses and gearing.

## Open items

- `simgui-ds.json` is uncommitted: arrow-key POV + H on Keyboard 0, and slot 1 changed from
  Keyboard 1 to Keyboard 0. Arrow keys are not yet confirmed working by hand. Commit after that.
- Nobody has driven the elevator by hand in the Sim GUI yet (only the auto and the headless run).
- The 3D model is a generated stand-in, not real CAD. `ENABLED_ON_REAL_ROBOT = false`, and the
  CAN ID and DIO port are placeholders.
- Nothing is pushed. Pushing and opening a PR needs approval.
