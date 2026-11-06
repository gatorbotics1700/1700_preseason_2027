# Subsystem template (one motor at a set speed)

Starter files for a new mechanism. "Template" stands for your mechanism's
name. Every file here is part of one subsystem: the subsystem itself plus
its hardware layer (IO).

This is the simplest shape: one TalonFX motor run at a speed, like a roller.
It builds and its tests run, but it is not added to RobotContainer, so it
never runs on the robot.

## The files

| File | What it is |
|---|---|
| TemplateIO.java | The interface: what the subsystem reads (inputs) and what it can tell the hardware to do. `new TemplateIO() {}` is the "does nothing" version. |
| TemplateIOTalonFX.java | The real motor. Settings are sent once, with retry. |
| TemplateIOSim.java | Pretend motor for tests and the simulator. Remembers what it was told. |
| TemplateSubsystem.java | Decisions only. Reads inputs, logs, updates the disconnect alert. |
| src/test/.../template/TemplateSubsystemTest.java | 3 example tests. They run instantly. |

## See it for real: the intake

The intake (subsystems/mech/) is built the same way and runs on the robot.
Open the files side by side to see what a finished mechanism looks like.

| Template | Intake |
|---|---|
| TemplateIO | IntakeIO: the same idea, with 4 motors and 2 limit switches |
| TemplateIOTalonFX | IntakeIOTalonFX |
| TemplateIOSim | IntakeIOSim |
| TemplateSubsystem | IntakeSubsystem: adds the deploy arm's position control, stop rules and homing |
| TemplateSubsystemTest | IntakeRollerTest and IntakeSubsystemTest |
| Constants snippet below | IntakeConstants in Constants.java (search for `class IntakeConstants`) |
| Config snippet below | the `mech.intake_*` lines in config_alecto.properties |

For position control and homing (arm, hood, turret), study the intake's
deploy arm. This template does not include it.

## How to use it

In these steps, `<Name>` is your mechanism's name, such as Feeder. Pick a
name that is not already used. Do NOT use Intake or Hood: those files
already exist, and copying over them would replace the real code.

1. Make your own branch first (see CLAUDE.md, Part 1).
2. Copy the 4 .java files into your mechanism's folder, for example
   subsystems/mech/. Copy the test into the matching test folder,
   src/test/java/frc/robot/subsystems/mech/.
3. Rename each copied FILE: right-click it, choose Rename, and change
   Template to `<Name>` (TemplateIO.java becomes `<Name>IO.java`). Java
   needs each file name to match the class name inside it.
4. Change the TEXT: press Cmd+Shift+H. Type Template in the first box and
   `<Name>` in the second. Turn on Match Case (the Aa button). In "files to
   include", list only your copied files, so nothing else changes. Click
   Replace All.
5. Fix the first line of each copied file to match the new folder, for
   example: package frc.robot.subsystems.mech;
6. Move the placeholders out of `<Name>IOTalonFX.java` (search for
   "TODO: move"):
   - the CAN ID goes in the config file, read by Constants.java
   - the current limits go in Constants.java
   Use the snippets below.
7. Run WPILib: Test Robot Code. Your copied tests should pass.
8. Then follow the steps in CLAUDE.md, and fill in the other TODOs.

## Snippet: Constants.java

Add a nested class inside Constants.java, next to the others:

    public static final class <Name>Constants {
      // Wiring comes from the robot's config file.
      public static final int MOTOR_CAN_ID =
          RobotConfigLoader.getInt("mech.<name>_motor_can_id");

      // Design numbers, typed once. TUNED, NOT DERIVED (date): ask a mentor.
      public static final double STATOR_CURRENT_LIMIT_AMPS = 40;
      public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30;
    }

Then in `<Name>IOTalonFX.java`, use `<Name>Constants.MOTOR_CAN_ID` and the
two limits, and delete the placeholder lines.

The real intake version, in IntakeConstants:

    public static final int LEFT_INTAKE_MOTOR_CAN_ID =
        RobotConfigLoader.getInt("mech.intake_left_roller_can_id");
    ...
    public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 70;
    public static final double DEPLOY_STATOR_CURRENT_LIMIT_AMPS = 40;

(The intake has no supply limits yet. The template adds them on purpose.)

## Snippet: config file

Add this to src/main/deploy/configFiles/config_alecto.properties (the only
config file in use), with your motor's real CAN ID:

    # <Name>
    mech.<name>_motor_can_id=<your CAN ID>

The real intake version:

    # Intake motors: CAN IDs on the mech CAN bus
    mech.intake_left_roller_can_id=8
    mech.intake_right_roller_can_id=11
    ...

## Known gaps

- If the motor rejects its settings 5 times in a row, it gives up quietly,
  with no alert (the shared PhoenixUtil.tryUntilOk helper).
- The sim motor does not move, so speed and current read 0 in the
  simulator. Physics comes later (see mechanism-checklist.md).
- No commands, no button bindings, and RobotContainer does not choose the
  IO by mode yet. Ask a mentor before wiring it in.
