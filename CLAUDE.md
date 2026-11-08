# Team 1700 robot code: instructions for Claude Code

This file has two parts.
- **Part 1 is for people** (students and mentors). It says how to start and what to type.
- **Part 2 is for Claude Code.** It says who you are helping, the rules for the code, and the step-by-step plan for cleaning up a mechanism.

The **Intake** was the first mechanism done this way. Use it as the worked example (see "Worked example: the Intake" near the end).

---

# PART 1: Start here (students and mentors)

## Set up, once

1. Open the Claude app and click the **Code** tab. (Don't have it? Download it from claude.com/download. Claude Code needs a paid Claude plan.)
2. Click **New session** and choose the `1700_preseason_2027` folder. That's the folder on your computer where the code was cloned.
3. Make your own branch (your own copy of the work). In VS Code:
   1. Click the branch name in the bottom-left corner.
   2. Click **Create new branch**.
   3. Name it `<mechanism>-<your name>`, for example `hood-maya`.
4. In Claude, type this and press Enter:

   > Read CLAUDE.md. I am working on the **\<mechanism\>** on branch **\<branch\>**. Start at Step 0.

## Every time you sit down

- Check that VS Code shows **your branch** in the bottom-left corner, not `main`.
- Type: **"Where are we, and what is the next tiny step?"**
- Claude asks before it changes a file. Read what it shows you. If you're not sure, say **"explain that first"**.
- You can always type **"stop"** or **"undo that"**.

## If something looks scary

- Red errors in the Problems tab that mention `AutoLogged` are normal until the project has been built once. Ignore them.
- Yellow warnings in the terminal during the simulator are normal.
- If the build says **BUILD FAILED**, copy the last 20 lines of the terminal and paste them to Claude.
- After a build, some files may show as changed even though you didn't touch them. The build tool fixes spacing automatically. A diff that is only spacing is fine.

---

# PART 2: Instructions for Claude Code

## Who you are helping

You are helping FRC Team 1700 (Gatorbotics). The person may be a **high school student** or a **mentor**. Many mentors haven't coded in a long time.

**Talk at the level of a high schooler who has taken one or two Java classes.**
- Use plain English. Explain every term the first time you use it, in one sentence.
- Go **one tiny step at a time**, one file per step unless they say otherwise.
- Tell them the exact clicks or commands. Never assume they know a shortcut.
- Before you change anything, say what you'll change, why, and how to undo it. Then wait for their OK.
- After each step, tell them how to check that it worked.
- Don't commit, push, delete files or install anything without asking first.
- If they seem lost, slow down and back up. Don't pile on more ideas.

## Who the code is for

The code must be easy for students to read. **Keep it simple.** Students have limited time, and every clever trick is something they'll have to learn before they can fix a bug at competition.

**Allowed:** classes, methods, fields, `if/else`, simple `for` loops, enums, and one interface per IO layer (explain it in a comment).

**Avoid:**
- generics you define yourself (`<T>`), streams, reflection, recursion, and inheritance deeper than one level
- abstract base classes and design patterns beyond the IO layer
- lambdas bigger than the small ones already in the repo
- builder chains or "clever" one-liners. Write three plain lines instead.

**Style:**
- Keep methods short, about 20 lines or fewer, with one job each.
- Put units in names: `Degrees`, `Rotations`, `Volts`, `Amps`, `Rps`, `Seconds`.
- Name every number. Constants are `LIKE_THIS`.
- Above each method, write one plain sentence about what it does. Comments explain **why**, not what.
- Mark any guessed number: `// TUNED, NOT DERIVED (date)`.
- No `static` for settings that can change. Use normal fields. `static final` is fine for numbers that never change.
- Match the style already in the repo. Don't reformat other files.
- Don't add libraries.

## Project facts

- **Repo:** `gatorbotics1700/1700_preseason_2027`. Work on the person's own branch. **Never commit to `main`.**
- **Stack:** Java, WPILib 2026.2.1, command-based, AdvantageKit logging, CTRE Phoenix 6 (TalonFX motors).
- **Folders** (all under `src/main/java/frc/robot/`):
  - `Robot.java`: the mode methods (disabled, auto, teleop). `robotPeriodic()` runs the scheduler.
  - `RobotContainer.java`: builds the subsystems and holds the button bindings.
  - `Constants.java`: holds one **nested class** per mechanism, such as `IntakeConstants` and `HoodConstants`. (A nested class is a small class written inside another class. There is no separate `IntakeConstants.java` file. Search `Constants.java` for `class IntakeConstants`.)
  - `subsystems/mech/`: intake, shooter, hood, turret, hopper.
  - `commands/mech/`: the actions, such as `IntakeCommands.java`.
  - `util/RobotConfigLoader.java`: reads the per-robot config file.
  - `subsystems/template/`: starter files for a **new** mechanism (one motor at a set speed). Not used by the robot. See its `README.md`.
  - `util/SimSmoke.java`: runs one auto in the simulator with no window, for `./gradlew simSmoke`.
  - `util/TunableGains.java` (dashboard gains, sent only when changed) and `util/LoopTimer.java` (counts loops for timeouts and alerts): shared helpers for any mechanism.
- **Test folders** (under `src/test/java/frc/robot/`): `subsystems/mech/` (mechanism tests), `diagnostics/` (log checks, one small class per problem; `PositionChecks` gives any position mechanism the standard ones), `view3d/` (3D models built from the constants, and their checks).
- **Config files:** `src/main/deploy/configFiles/`. **Only `config_alecto.properties` is in use.** The others (`hulk`, `sting`, `experimental`) are for retired robots. New keys go in `config_alecto.properties` only.
- **Build and run (in VS Code):** press Cmd+Shift+P, then:
  - `WPILib: Build Robot Code` to build
  - `WPILib: Test Robot Code` to run the tests
  - `WPILib: Simulate Robot Code`, then pick the Sim GUI, to run the simulator
- **Build from the terminal (Mac):** `JAVA_HOME=~/wpilib/2026/jdk ./gradlew build --offline`. This builds, and runs the tests where they are turned on.
- **Sim smoke and log checks:** in VS Code, Cmd+Shift+P > `Tasks: Run Task` > a `Gatorbotics:` task (`.vscode/tasks.json`). The same from a terminal (any OS; `gradlew.bat` on Windows):
  - `./gradlew simSmoke` runs the whole robot in the simulator with no window, runs one auto (`-PsimSmokeAuto="<name>"`, default "B LN-PL"), then checks its log. It fails on any problem. Log and console: `logs/sim_smoke/`.
  - `./gradlew diagnoseLog -Plog=<file or folder>` runs the same checks on any log, such as a match log, and prints a report.
  - Before asking for review: tasks `Gatorbotics: Run code tests` and `Gatorbotics: Smoke test sim` (`./gradlew build simSmoke`). How it all fits: `docs/3d-sim-workflow.md`.
- **Tests** live in `src/test/java/frc/robot/`. **On `main`, tests are compiled but not run yet:** the `test { ... }` block in `build.gradle` is commented out. The `intake-nidhi` and `mentors/szymon/sim-framework` branches turn them on (and run each test class in its own fresh Java process, `forkEvery = 1`). Don't turn them on in another branch without asking a mentor; the team will decide when to turn them on for everyone.
- **Don't touch** other mechanisms, the `RobotContainer` layout, or anything outside the person's mechanism unless they ask.
- **Delete superseded in-between code** (old tests, temporary helpers). Don't keep it around. For large pieces of **working robot code** that is being replaced, ask first. The team may want an archive tag.

## Lessons the team learned (follow these)

- **Keep the 20 ms loop light.** Never call heavy code from `periodic()` or from a logging method. Send motor settings only when they change.
- **No wall-clock time or random numbers in robot code.** For "has it been N seconds?", count `periodic()` loops (one loop = 20 ms). That way, replay and tests give the same answer, and tests don't have to wait.
- **Normal fields, not `static`, for settings.** `static` settings are shared between copies of a class, and tests build many copies.
- **Where each number lives:**
  - **Wiring** (CAN IDs, DIO ports) goes in the config file. The mechanism's constants class reads it with `RobotConfigLoader.getInt("mech.<name>")`, the way `HoodConstants` does.
  - **Design numbers** (current limits, angles, gear ratios) go in the mechanism's constants class, **typed once**. The real IO and the sim IO both read them from there.
  - **Tuning gains** go on the dashboard (`LoggedNetworkNumber`), with the default in one place.
- **Every config key must be read by code.** A key nothing reads is a trap: people edit it and nothing happens.
- **Don't turn a mechanism off with a fake CAN ID** (the turret uses 100). Use the "does nothing" IO version.
- **Game-specific logic stays out of the mechanism's file.** For example, "drive to the tower" belongs with navigation.
- **Three questions for every check or alert:** Is it connected and healthy? Does it agree with itself? What was it told to do, and does it move?
- **Limit switches:** an unwired roboRIO DIO port reads `true`. Check with the build team which way each switch reports.
- **A stop rule at the same current as the motor's own current limit may never trip**, because the motor is held at that limit.
- **Long checklist:** `mechanism-checklist.md` has the full mentor version. Read it when asked for a checklist review.
- **Simulator, 3D and tuning lessons** are at the end of `docs/3d-sim-workflow.md`: work out feedforward from the motor model, look at the current limit before the gains, check layout keys against a real log.

## The plan for a mechanism (tiny steps)

**Every mentor and student follows these same steps, in this order, for any mechanism** (hood, shooter, turret, hopper, climber...). The Intake went through them first; see the worked example below.

**Tests come before any refactoring.** Steps 1 and 2 change no robot behavior: read the code, then add a smoke test and tests of what the mechanism does today. Only then start moving code (Step 3). If a test breaks during the refactor, the refactor changed behavior. Don't skip Step 2, even if the mechanism looks simple.

Every step ends with: the project builds, the tests pass, the simulator runs, and the person can explain what changed. Before each step, explain the plan in plain words and wait for their OK.

- **Step 0: Set up.** Confirm the branch. Look at any files that already show as changed, and help discard spacing-only changes.
- **Step 1: Read, no changes.** Explain the mechanism's subsystem file in plain language, one section at a time, and answer questions. Start a **findings list** of anything that looks wrong. Don't fix findings yet. Add `TODO (date), CHECK WITH TEAM` comments.
- **Step 2: Safety-net tests.** Before refactoring, add a smoke test (the robot starts) and a few tests of current behavior, so the refactor can't silently change it.
- **Step 3: IO layer, part 1.** For a brand-new mechanism, start by copying `subsystems/template/` (see its `README.md`). For an existing one, add an `XIO` interface with `@AutoLog` inputs for the simplest part, for example the rollers. Add three versions: the real one (`XIOTalonFX`, with the motor code moved over **unchanged**), a sim one (`XIOSim`, which just remembers what it was told and has test setters), and a do-nothing one (`new XIO() {}`). The subsystem keeps only decisions and reads hardware only through `inputs`.
- **Step 4: IO layer, the rest.** Do the same for the remaining hardware (arm, sensors). Rewrite the tests to use `XIOSim`, so they run instantly with no waiting.
- **Step 5: Config.** CAN IDs and ports go into `config_alecto.properties`, read through the constants class. Design numbers are typed once in the constants class.
- **Step 6: Alerts.** A Driver Station `Alert` for each motor that disconnects. An alert if the mechanism was told to move but hasn't arrived after N seconds (counted in loops). One test per alert.
- **Step 7: Small math tests.** Unit conversions (degrees ↔ motor rotations) and the stop or decision rules.
- **Step 8: Simulator physics, 3D view and log checks.** Follow `docs/3d-sim-workflow.md`; the elevator (branch `mentors/szymon/elevator-prototype`) is the worked example. The physics numbers (length, mass, gearing, travel limits) must come from the build team; until then mark them TUNED, NOT DERIVED.
- **Step 9: Notes.** A one-page tuning note for the wiki, the findings list for the team, and the checklist below ticked.

**Testing recipe (used in Steps 4, 6 and 7):**
- Build a fresh `XIOSim` and subsystem for each test.
- Call `periodic()` directly in a loop. Never sleep or wait.
- Type the expected numbers in the test itself, not read from constants, so an accidental change makes a test fail.
- After writing a test, break the code on purpose once and check that the test fails, then put it back.

## Simulator and visualization lessons

**What the simulator shows today (no physics yet):**
- The sim IO (`XIOSim`) only remembers what it was told. Nothing moves by itself, so currents and positions stay 0. That is expected until Step 8.
- In the Sim GUI, an unwired DIO port reads `true`, so limit switches can look "pressed". Click them in the DIO panel to test the switch logic by hand.
- A move alert ("told to move but has not arrived") **will** fire in the sim after N seconds, because the pretend arm never arrives. Click the matching switch to "arrive".
- Keyboard 0 starts with only 4 buttons. To use more (for example the right bumper, button 8), raise the button count in Keyboard 0 Settings and assign a key.
- **Mac laptops** have no numpad for the D-pad: in DS > Keyboard N Settings, map the POV (D-pad) to the arrow keys. The Mac simulator reads a controller's Back button as button 11, so raise the button count to 11 and give it a key.

**Watching and replaying in AdvantageScope:**
- Live: connect to the sim and set Live Mode to **Logging**, not Low Bandwidth, or the export is incomplete.
- **Playback** (works today): File → Export Data → WPILOG, then open it later. Good for demos and reviews.
- **Sim smoke logs** (`logs/sim_smoke/`) can be opened directly: File → Open Log(s).
- **True AdvantageKit replay** (re-running the code on an old log) also needs `RobotContainer` to pick the do-nothing IO for every mechanism in replay mode. Not set up yet; ask before touching those files.

**Physics (Step 8):**
- Use the WPILib model that fits the shape: `SingleJointedArmSim` (arm, hood), `ElevatorSim` (elevator, climber), `FlywheelSim` or `DCMotorSim` (rollers, turret).
- Its numbers (mass, length, gearing, travel limits) must be **measured by the build team**, not copied from the code's constants. Otherwise the simulator only agrees with the code.
- Drive the simulated sensors (limit switches, encoders) from the simulated position, so homing and stop rules run on their own.
- Write down what the sim cannot show: jams, binding, real current under load.

**Pictures of the mechanism:**
- **2D (`Mechanism2d`):** a simple stick drawing (for example, a line that swings out when the intake deploys). Needs no build-team numbers, only the angle the code already knows. Easy first visual.
- **3D (AdvantageScope 3D field):** many top teams (6328 and others) log each moving part as a `Pose3d` (where it is and how it is turned) and load a 3D robot model into AdvantageScope. You then watch the real robot, or the sim, move in 3D: the turret turning, the hood tilting, the intake swinging out. Especially useful for a **turret**, where "which way is it pointing" is hard to see from numbers. Needs: a robot model (from CAD, or a stand-in built from boxes) and one pose per moving part. **Worked example:** the elevator on branch `mentors/szymon/elevator-prototype`; follow `docs/3d-sim-workflow.md`. Next candidate: the turret.

## Worked example: the Intake (branch `intake-nidhi`)

**Files:**
- `subsystems/mech/IntakeSubsystem.java`: decisions only
- `IntakeIO.java`, `IntakeIOTalonFX.java` (the real hardware), `IntakeIOSim.java` (the pretend hardware)
- `commands/mech/IntakeCommands.java`
- `IntakeConstants` inside `Constants.java`
- Tests: `src/test/java/frc/robot/subsystems/mech/IntakeRollerTest.java` and `IntakeSubsystemTest.java`

**Devices:**
- 4 TalonFX motors on `TunerConstants.mechCANBus`: left roller 8, right roller 11, left deploy 9, right deploy 10
- 2 limit switches on roboRIO DIO ports 3 (retracted) and 4 (deployed)
- The CAN IDs and ports are in `config_alecto.properties` as `mech.intake_*`.

**How it works:**
- The rollers run at a percent output.
- The deploy arm uses Motion Magic Expo position control. It stops at a limit switch, within 3° of its target, or at the jam current.
- It re-zeroes its angle at a limit switch.

**Alerts:**
- one per motor for "disconnected"
- "arm was told to move but has not arrived", after 2 s (100 loops)

**Done so far:** Steps 0–6.

**Findings for the team:**
- switch polarity
- `home()` runs every loop
- the jam current equals the motor's current limit
- the 70 A vs 90 A roller limit, applied to the right roller only
- the tuning defaults disagree with the "vaguely works" notes
- the SysId margin and the 18 V test
- `setDesiredDeployPosition(true)` means retract
- the unused `DEPLOYED_CURRENT_LIMIT`
- the unused `mech.robot_to_intake_yaw_degrees` key
- the robot assumes it starts retracted

**Simulator controls:**
1. Drag Keyboard 0 onto `Joystick[0]`.
2. Click Disabled, then Teleoperated.
3. Keys:
   - **X** runs the intake.
   - **V** reverses it.
   - **B** (after mapping button 8 in Keyboard 0 Settings) toggles deploy.
4. Watch `Mech/Intake/...` under NetworkTables, AdvantageKit.

## Definition of done (the team checklist)

This is the short student version. The long mentor version is in `mechanism-checklist.md`.

**Before you code**
- [ ] What the mechanism must do, and the method names others will call, are written down.
- [ ] Its shape is chosen (spins, moves to a position, or moves along a line).
- [ ] A date it should work by, with a weekly check.

**Build it**
- [ ] Hardware code is in its own IO class, with a simulated version and a version that does nothing.
- [ ] CAN IDs and ports are in the config file. Design numbers are typed once in the constants class.
- [ ] Numbers to tune show on the dashboard.
- [ ] Guessed numbers are marked TUNED, NOT DERIVED.
- [ ] The mechanism's file is short and holds only what is specific to it.

**Make problems easy to see**
- [ ] An alert if a motor or sensor disconnects.
- [ ] An alert if it was told to move but isn't getting there.
- [ ] Position mechanisms show homed status and don't move until homed.
- [ ] What it was told to do, and what it did, show up in the log.
- [ ] It is part of the pit check.

**Prove it works**
- [ ] It runs in the simulator on a laptop.
- [ ] Position mechanisms: a demo auto passes `./gradlew simSmoke`, and its log checks are in `diagnostics/`.
- [ ] The robot still starts (smoke test), and all tests pass. (The repo has no CI yet, so run `WPILib: Test Robot Code` before asking for review.)
- [ ] A few small tests cover the math and each alert.

**Hand it off**
- [ ] A one-page wiki note on what the numbers mean and how to tune them.
- [ ] A mentor reviews the pull request. The branch is deleted after merging.

## Git, simply

- Commit small, with one idea per commit and a plain message, such as "add comment listing intake devices".
- To commit in VS Code:
  1. Open the Source Control panel.
  2. Click `+` next to each file.
  3. Type a message.
  4. Click **Commit**.
- To upload, click **Publish Branch** (the first time) or **Sync**.
- Ask before every commit and push. Never force-push. Never delete a branch.
- Don't commit `simgui-ds.json` unless the team agrees. It holds one person's simulator keyboard setup.

## Words people might not know

- **Subsystem:** a class that owns one mechanism's hardware.
- **Command:** one action, like "deploy intake".
- **`periodic()`:** runs automatically every 20 milliseconds.
- **TalonFX:** the motor controller. Its **CAN ID** is how the code finds it.
- **IO layer:** a split between decisions (the subsystem) and hardware talk (a separate class), so a simulator or a test can stand in for real motors.
- **Interface:** a list of method names a class promises to have. The IO layer uses one, so the real, sim and do-nothing versions can be swapped.
- **`@AutoLog`:** a label that makes AdvantageKit write a matching `...AutoLogged` class during the build.
- **Alert:** a warning that shows on the dashboard while a problem is happening and disappears when it's fixed.
- **Nested class:** a class written inside another class, like `IntakeConstants` inside `Constants`.
- **Branch:** your own copy of the work. **Commit:** a saved snapshot. **Pull request:** a request for review.
