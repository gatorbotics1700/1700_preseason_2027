# Elevator + 3D sim: session brief

Branch `mentors/szymon/research`. Local commits only, nothing pushed. Last updated Nov 7 2026.

## Goal

Get the elevator onto main's mechanism patterns (template, CLAUDE.md steps), with a test for each
requirement, then build the simulator, 3D view and automatic checks on top of that architecture,
so the elevator proves a workflow every future mechanism can follow.

## Status: done

| Piece | Where |
|---|---|
| Elevator in main's pattern: IO layer, stand-in `ElevatorIOSim`, physics `ElevatorIOSimPhysics` | `subsystems/mech/Elevator*` |
| Requirements, each mapped to a test | `docs/elevator-requirements.md` |
| Sim smoke: any auto, headless, then log checks | `util/SimSmoke.java`, `./gradlew simSmoke` |
| Log diagnostics for any log | `src/test/java/frc/robot/diagnostics/`, `./gradlew diagnoseLog -Plog=...` |
| 3D view integrity (model from constants, layout, pose checks) | `src/test/java/frc/robot/view3d/` |
| Workflow guide for students | `docs/3d-sim-workflow.md` |
| Every attempt, including rejected ones | `docs/elevator-attempts.md` |

**Pass condition:** `./gradlew build simSmoke` ends with `BUILD SUCCESSFUL` (about 20 s, any OS).

**Also fixed on the way:** HubTrackerSubsystem rebuilt every loop; turret disabled with fake CAN
ID 100; stale robot pose key in the layout; sim robot started in a field corner;
`ElevatorPhysicsTest` passed only by test order; a WPILib 2026.2.1 `DataLogReader` bug (drops a
short last record; worked around in `LogFile`, worth reporting upstream).

## Next

1. Decide with the mentors what lands in `main` as team infrastructure and what stays on a
   prototype branch with the elevator.
2. Try `./gradlew diagnoseLog` on a real match log (CAN errors come from the console saved in the
   log; not tried on a real one yet).

## Open items

- Turret: replace its `ENABLED` flag with a do-nothing IO once it gets an IO layer.
- The elevator's model is a stand-in, its CAN ID and DIO port are placeholders, and
  `ENABLED_ON_REAL_ROBOT = false`. Real CAD needs the build team's export and measured masses.
- Tests are on in this branch (`forkEvery = 1`, like `intake-nidhi`); main still has them off.
- Nobody has driven the elevator by hand in the Sim GUI yet.
