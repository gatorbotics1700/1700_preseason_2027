# Mechanism checklist, full version (WIP)

Use it for every new mechanism, and for the mentors' "final good" versions that will seed the team's library, so students can copy a mechanism that already has the right shape. It comes from the Programming Review (recommendations 2 to 5) and from the 6328, 2910, 1678, 254, 9470 and Space Cookies code. Keep adding to it as the mechanisms get built.

**Order matters.** Every mentor follows the step-by-step plan in CLAUDE.md for every mechanism: read the code, then add a smoke test and tests of today's behavior **before refactoring anything**. If a test breaks during the refactor, the refactor changed behavior. The intake (branch intake-nidhi) is the worked example.

**How to use it.** Copy the list into the mechanism's wiki page and tick the boxes. Tags: **[alert]** runs on the robot and warns on the Driver Station, **[test]** is a unit test that runs on a laptop, **[checklist]** is something a person does or confirms. If a box does not apply, write n/a and a reason instead of skipping it.

## 0. Design first, before writing code
- [ ] Write the interface first: the methods the rest of the robot will call, and the values it reads back.
- [ ] Say what shape it is: speed (roller, flywheel), position (pivot, hood, turret) or linear (elevator, climber). Use the shared class for that shape once it exists. (None exists yet; a shared Roller class is planned after the intake.)
- [ ] Decide how it will be tested without the robot, and who builds the simulator.
- [ ] Agree a date by which it should work, and review progress at a short weekly check. If it is behind, cut scope early.
- [ ] List what is specific to this mechanism. Everything else (config, logging, SysId, alerts) should come from shared code.

## 1. Use AdvantageKit and the IO pattern
- [ ] An IO interface with @AutoLog inputs. Everything the mechanism reads from hardware comes through updateInputs.
- [ ] Three implementations: real (TalonFX), simulated, and disabled (does nothing).
- [ ] The subsystem takes the IO in its constructor and holds decisions only, with no motor classes in it.
- [ ] RobotContainer, or a subsystem factory, picks real, simulated or disabled by mode, the way Drive is built.
- [ ] Inputs go through Logger.processInputs so the mechanism can be replayed. Use the FPGA timestamp or loop counts, never System.currentTimeMillis or random numbers.
- [ ] Worked example: the intake (IntakeIO, IntakeIOTalonFX, IntakeIOSim, IntakeSubsystem) on the intake-nidhi branch.
- [ ] Instance fields, not static ones, except `static final` for values that never change (like CAN IDs). Tests and the IO pattern build several copies of a mechanism (real, sim, do-nothing, a fresh one per test), and a static field makes them silently overwrite each other.

## 2. Unit and smoke tests
- [ ] Before any refactor: a smoke test and tests of current behavior exist and pass. [test]
- [ ] The RobotContainer smoke test still passes with the mechanism added. [test]
- [ ] Decision rules are small functions that take plain numbers and return a number, with a test beside each. [test]
- [ ] Conversions are tested: units, gear ratios, encoder to angle, speed to flywheel speed (round trip). [test]
- [ ] Each alert is tested by feeding in a fake bad input. [test]
- [ ] For position mechanisms, a test that nothing is commanded before homing. [test]
- [ ] The tests run in CI. (The repo has no CI yet, so run WPILib: Test Robot Code before every review.) Aim for the checks that would hurt if wrong, not a coverage number.
- [ ] Tests use the sim IO: a fresh IO and subsystem per test, call periodic() directly, count loops, and never sleep. The intake's 19 tests run in about 0.06 s. [test]
- [ ] Expected numbers are typed in the test, not read from constants, so an accidental change fails a test. [test]
- [ ] Each new test was proven by breaking the code on purpose once and watching it fail. [checklist]

## 3. Logging
- [ ] Setpoints, actual values, state, home status and the running command are all logged.
- [ ] Log keys follow one pattern, such as Mechanism/Name/Value.
- [ ] Open a real or simulated log in AdvantageScope and confirm the values look right. [checklist]
- [ ] The mechanism has a saved panel in the team's AdvantageScope layout.
- [ ] Know the difference: log **playback** (AdvantageScope, Live Mode set to Logging, then Export Data) works today. True AdvantageKit **replay** also needs the simulator to write a WPILOG file in Robot.java, and RobotContainer to pick the do-nothing IO in replay mode.

## 4. Checks and alerts

**a. Is it connected and healthy?**
- [ ] A Driver Station alert for every motor and sensor that drops off the bus. Not debounced for now, to keep it simple (the intake and the template match); add a short debounce if it flickers on the real robot. [alert, test]
- [ ] Temperature and fault warnings: undervoltage, hardware faults, over-temperature. [alert]
- [ ] Stator and supply current limits are set on every motor. No supply limits exist in the code today. [checklist]
- [ ] Robot-level, built once: a low battery warning in the pit, and CAN bus error counts. [alert]

**b. Does it agree with itself?**
- [ ] Asked for a speed or position but not getting it for N seconds raises a named alert. Count periodic() loops (1 loop = 20 ms), not clock time, so replay and tests agree. [alert, test]
- [ ] Two sources for the same value agree, such as motor angle and absolute encoder, or vision position and wheel odometry. [alert, test]
- [ ] Position mechanisms show homed status, and nothing moves until homed. [alert, test]
- [ ] Every setting was accepted by the controller (retry, then alert), and the right config file was loaded. [alert, test]

**c. What was it told to do, and does it move?**
- [ ] Setpoints and the running command are logged. [checklist]
- [ ] An entry in the pit system check: one button, robot on blocks, each part moves briefly, pass or fail on the dashboard. [alert, checklist]

## 5. Simulator
- [ ] A simulated IO exists, with a physics model that fits the shape: WPILib's arm, elevator or flywheel simulation, or the real controller run in simulation.
- [ ] The model's numbers (mass, length, gearing, limits, friction) are measured on the real mechanism, not copied from the constants the code uses. Otherwise the simulator only agrees with the code.
- [ ] Simulated sensors come from the simulated state: limit switches, hall effect, absolute encoder with its wraparound. That lets homing and stop logic run.
- [ ] A scripted scenario runs on a laptop and prints a report: reaches the target in a reasonable time, homes from random starting positions, feeds only when ready.
- [ ] Faults can be injected in simulation (unplug a motor or encoder) and the alert fires. [test]
- [ ] A short note says what the simulator cannot show: jams, binding, real current under load.
- [ ] Before physics exists, people know what to expect: the sim IO only remembers commands, so nothing moves and currents stay 0; unwired DIO switches read true (click them in the Sim GUI DIO panel); "not arrived" alerts will fire. [checklist]
- [ ] Simulator controls are written on the mechanism's page: which keyboard keys do what. Keyboard 0 starts with 4 buttons; raise the count in Keyboard 0 Settings for more. [checklist]

**Pictures of the mechanism**
- [ ] A 2D Mechanism2d drawing (for example a line that swings when the intake deploys). Needs only the angle or height the code already knows, no build-team numbers.
- [ ] Optional, strongly suggested for a turret or hood: a 3D view in AdvantageScope. Log each moving part as a Pose3d and load the robot's CAD model from the build team, the way 6328 does. You then watch the turret turn or the hood tilt in 3D, live or in a log, which makes "which way is it pointing?" obvious.

## 6. Util libs
- [ ] Use the shared helpers, and add to them instead of copying code: one alliance helper, alert and debounce helpers, tunable numbers, config apply with retry, SysId setup.
- [ ] Units are typed or in the names (meters, degrees, rotations), with one place for each conversion.
- [ ] Every speed and position is named by its frame, such as robotRelativeSpeeds and fieldRelativeSpeeds, wherever the mechanism uses them.
- [ ] If the same lines appear in two mechanisms, move them to shared code.

## 7. Documentation
- [ ] A one-page tuning note in the wiki: what each number means and how it was found. [checklist]
- [ ] Any guessed number is marked TUNED, NOT DERIVED, with the date and robot.
- [ ] Commands: one line on what each command does. States: what they are and how it moves between them.
- [ ] How to run the simulator and the pit check, on the mechanism's page.
- [ ] A short why-this-works note wherever the code works and nobody knows why.
- [ ] If a camera is involved: measure the mount, enter it in the config file, check it in AdvantageScope, and record who and when.

## 8. Right level of abstraction
- [ ] The mechanism's own file holds only what is specific to it. A test: could next year's version be a short configuration?
- [ ] Game-specific logic, such as driving to the tower or aligning to a target, lives with navigation or autos, not in the mechanism's commands.
- [ ] Two copies of the mechanism would work side by side.
- [ ] It is clear what a student changes for a new game, and what they should not touch.

## 9. Config separate from code, and tunable
- [ ] Wiring (CAN IDs, sensor ports, encoder offsets) is in the config file. The mechanism's constants class (a nested class in Constants.java, such as IntakeConstants) reads it with RobotConfigLoader, the way HoodConstants does.
- [ ] Only config_alecto.properties is in use. hulk, sting and experimental are retired robots. A missing key stops startup, and the smoke test catches it.
- [ ] Every config key is read by code. A key nothing reads is a trap (example: mech.robot_to_intake_yaw_degrees is in the files, but the code uses a typed 180).
- [ ] Design numbers (current limits, angles, gear ratios) are typed once in the constants class. The real IO and the sim IO both read them there, so they can't drift apart.
- [ ] A mechanism is switched off with its do-nothing IO version, not a fake CAN ID (the turret uses 100 today).
- [ ] An alert warns when the config loader falls back to the default file for an unknown roboRIO serial. [alert]
- [ ] Gains, speeds and limits are tunable on the dashboard, with the defaults in one place.
- [ ] Shared physical facts (gear ratio, travel limits, mass) live in the mechanism's constants, with units in the names.

## 10. Mechanism-specific extras

**Position (turret, hood, elevator, climber)**
- [ ] Home against a hard stop, a limit switch, or an absolute encoder whose ratio and offset were measured, and show the home status.
- [ ] Soft limits are set, and the wraparound or travel logic is tested.
- [ ] The starting assumption is written down (example: the intake assumes it starts retracted), and what happens if it is wrong.
- [ ] Turret: measure once how many times the encoder turns per turret turn.

**Speed (shooter, rollers)**
- [ ] Spin-up reaches speed within a set time, and feeding waits for it.
- [ ] The speed to exit speed conversion is tested, and the slip number is measured, not guessed.

## 10b. Common gotchas (found in the intake)
- [ ] An unwired roboRIO DIO port reads true. Confirm with the build team which way each switch reports "pressed".
- [ ] Motor settings are sent only when they change, not every loop. Re-homing (setPosition) also happens only on a change, not 50 times a second.
- [ ] A jam or stall rule set at the same current as the motor's own current limit may never trip. Put the rule below the limit.
- [ ] A setting meant for a pair of motors is applied to both (the intake's live roller limit only reaches the right roller).
- [ ] Dashboard tuning defaults match the values the robot actually ran with.
- [ ] Method names read the right way round (setDesiredDeployPosition(true) means retract).
- [ ] SysId limits sit inside the real travel, and test voltages are reasonable (the intake's dynamic test uses 18 V).
- [ ] Brake or coast mode is set on purpose, not left at the default.

## 11. Before merging
- [ ] A short-lived feature branch, one review, and all tests passing (CI, once the repo has it).
- [ ] Anything large being deleted gets an archive tag and a one-line reason.
- [ ] The branch is deleted after merging.
- [ ] The final version is tagged for the library, such as library/shooter-v1.
- [ ] Superseded in-between code (old tests, temporary helpers) is deleted, not kept.
- [ ] simgui-ds.json (one person's simulator keyboard setup) is left out unless the team agrees.
- [ ] The wiki page, tuning note and this checklist are done and linked from the pull request.

## Code style for students
- [ ] Readable by a student with one or two Java classes: no custom generics, streams, recursion, reflection, deep inheritance or clever one-liners. Short methods, units in names, one plain comment per method.

## Reference code to read
- Speed and generic classes: 6328's subsystems/rollers (RollerSystem with IO and Sim), launcher/flywheel and launcher/hood; 2910's subsystems/base (MotorIO, SimMotorIOTalonFX, roller and servo classes).
- Position: 1678's hood, intakedeploy and climber folders (shared ServoMotorSubsystem base); 6328's launcher/hood, which zeroes against a stop; 254's 2022 Turret.java, which homes with a limit switch on a shared base class.
- Elevator: Space Cookies' TagalongLib Elevator, ElevatorIO and configuration (BSD 3-Clause: keep the notice if code is copied); the student elevator branches in the preseason repo, with myraElevator the best starting point.
- Climber: 1700's 2025 ClimbingSubsystem in Robot_2025 (25 lines, open loop); the 2026 ClimberSubsystem is in the 1700_Robot_2026 history before the March 17 commit; 1678's climber.
- Intake, hopper and shooter: BREAD's hood, intake, juicer, shooter, treadmill and tunnel folders; 2910's intake, hopper and shooter.
- Testing style: 9470's tests that pull decisions into small functions (ShooterTest, SuperstructureTest); 6328's 15-line RobotContainerTest.

## Open questions (WIP)
- How realistic does a simulator need to be: setpoints applied instantly (2910) or the real controller run in simulation?
- What are the right current limits? No supply limits are set today.
- Does AdvantageKit already log CAN bus errors, or does that need adding?
- What caused the motor brownout at CalGames?
- When is a mechanism final good enough to seed the library, and who signs it off?
- Is main protected in GitHub, and should the robot repo get CI? (There is none today.)
- Should a "told to move but not arriving" alert also stop the mechanism, or only warn?
