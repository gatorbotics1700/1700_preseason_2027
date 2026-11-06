# Subsystem template (one motor at a set speed)

Starter files for a new mechanism. "Template" stands for your mechanism's
name. Every file here is part of one subsystem: the subsystem itself plus
its hardware layer (IO).

This is the simplest shape: one TalonFX motor run at a speed, like a roller.
For position control and homing (arm, hood, turret), also study the intake's
deploy arm in subsystems/mech/IntakeSubsystem.java.

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

## How to use it (example: a hood)

1. Make your own branch first (see CLAUDE.md, Part 1).
2. Copy the 4 .java files into your mechanism's folder, for example
   subsystems/mech/. Copy the test into the matching test folder,
   src/test/java/frc/robot/subsystems/mech/.
3. Rename each copied FILE: right-click it, choose Rename, and change
   Template to Hood (TemplateIO.java becomes HoodIO.java). Java needs each
   file name to match the class name inside it.
4. Change the TEXT: press Cmd+Shift+H. Type Template in the first box and
   Hood in the second. Turn on Match Case (the Aa button). In "files to
   include", type the folder you copied into, so nothing else changes.
   Click Replace All.
5. Fix the first line of each copied file to match the new folder, for
   example: package frc.robot.subsystems.mech;
6. Move the placeholders out of HoodIOTalonFX.java (search for "TODO: move"):
   - the CAN ID goes in the config file, read by Constants.java
   - the current limits go in Constants.java
   Use the snippets below.
7. Run WPILib: Test Robot Code. Your copied tests should pass.
8. Then follow the steps in CLAUDE.md, and fill in the other TODOs.

## Snippet: Constants.java

Add a nested class inside Constants.java, next to the others:

    public static final class HoodConstants {
      // Wiring comes from the robot's config file.
      public static final int MOTOR_CAN_ID =
          RobotConfigLoader.getInt("mech.hood_motor_can_id");

      // Design numbers, typed once. TUNED, NOT DERIVED (date): ask a mentor.
      public static final double STATOR_CURRENT_LIMIT_AMPS = 40;
      public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30;
    }

Then in HoodIOTalonFX.java, use HoodConstants.MOTOR_CAN_ID and the two
limits, and delete the placeholder lines.

## Snippet: config file

Add this to src/main/deploy/configFiles/config_alecto.properties (the only
config file in use), with your motor's real CAN ID:

    # Hood
    mech.hood_motor_can_id=<your CAN ID>

## Known gaps

- If the motor rejects its settings 5 times in a row, it gives up quietly,
  with no alert (the shared PhoenixUtil.tryUntilOk helper).
- The sim motor does not move, so speed and current read 0 in the
  simulator. Physics comes later (see mechanism-checklist.md).
- No commands, no button bindings, and RobotContainer does not choose the
  IO by mode yet. Ask a mentor before wiring it in.
