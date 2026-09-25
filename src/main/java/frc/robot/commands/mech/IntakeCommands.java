package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommands {

  public IntakeCommands() {}

  /**
   * Open-loop deploy duty toward the retract hall until it trips, then sync {@code
   * deployGoalExtended} so {@link IntakeSubsystem#periodic()} holds motor off at the hall.
   */
  private static Command seekUntilRetractSwitch(IntakeSubsystem intakeSubsystem) {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              if (intakeSubsystem.isRetractedLimitSwitchTriggered()) {
                intakeSubsystem.zeroIntakeDeploy(true);
              } else {
                intakeSubsystem.setDeploySpeed(IntakeConstants.HOMING_SPEED);
              }
            },
            intakeSubsystem),
        Commands.deadline(
                Commands.waitUntil(intakeSubsystem::isRetractedLimitSwitchTriggered)
                    .withTimeout(10),
                Commands.run(
                    () -> intakeSubsystem.setDeploySpeed(IntakeConstants.HOMING_SPEED),
                    intakeSubsystem))
            .finallyDo(intakeSubsystem::clearDeployManualControl),
        new InstantCommand(() -> intakeSubsystem.setDeployGoalExtended(false), intakeSubsystem));
  }

  /**
   * Open-loop deploy duty toward the deployed hall until it trips, then sync goal so periodic holds
   * at the hall.
   */
  private static Command seekUntilDeployedSwitch(IntakeSubsystem intakeSubsystem) {
    double speed = -IntakeConstants.HOMING_SPEED;
    return Commands.sequence(
        new InstantCommand(
            () -> {
              if (intakeSubsystem.isDeployedSwitchEffectTriggered()) {
                intakeSubsystem.zeroIntakeDeploy(false);
              } else {
                intakeSubsystem.setDeploySpeed(speed);
              }
            },
            intakeSubsystem),
        Commands.deadline(
                Commands.waitUntil(intakeSubsystem::isDeployedSwitchEffectTriggered)
                    .withTimeout(10),
                Commands.run(() -> intakeSubsystem.setDeploySpeed(speed), intakeSubsystem))
            .finallyDo(intakeSubsystem::clearDeployManualControl),
        new InstantCommand(() -> intakeSubsystem.setDeployGoalExtended(true), intakeSubsystem));
  }

  /** Run open-loop toward the retract it until it trips (startup / homing). */
  public static Command HomeIntake(IntakeSubsystem intakeSubsystem) {
    return seekUntilRetractSwitch(intakeSubsystem).withName("Home Intake Retract");
  }

  public static Command ToggleIntake(IntakeSubsystem intakeSubsystem) {
    if (intakeSubsystem.getIsDeployed().getAsBoolean()) {
      return RetractIntake(intakeSubsystem);
    } else {
      return DeployIntake(intakeSubsystem);
    }
  }

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return seekUntilRetractSwitch(intakeSubsystem).withName("Retract Intake");
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return seekUntilDeployedSwitch(intakeSubsystem).withName("Deploy Intake");
  }

  public static Command RunIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
            () -> {
              intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKING_SPEED);
            })
        .withName("Run Intake");
  }

  public static Command ReverseIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
            () -> {
              intakeSubsystem.setIntakeSpeed(-IntakeConstants.INTAKING_SPEED);
            })
        .withName("Reverse Intake");
  }

  public static Command StopIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
            () -> {
              intakeSubsystem.setIntakeSpeed(0);
            })
        .withName("Stop Intake");
  }
}
