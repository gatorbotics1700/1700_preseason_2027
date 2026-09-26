package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommands {

  public IntakeCommands() {}

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
            .finallyDo(() -> intakeSubsystem.setDeploySpeed(0)));
  }

  public static Command ToggleIntake(IntakeSubsystem intakeSubsystem) {
    if (intakeSubsystem.isDeployed()) {
      return RetractIntake(intakeSubsystem);
    } else {
      return DeployIntake(intakeSubsystem);
    }
  }

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new RetractIntakeCommand(intakeSubsystem)
        .andThen(new HomeIntakeRetract(intakeSubsystem))
        .andThen(
            Commands.waitSeconds(0.75)
                .alongWith(Commands.run(() -> {}, intakeSubsystem))
                .withName("Intake Sequence Wait"))
        .withName("Retract Intake");
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(intakeSubsystem)
        .andThen(new HomeIntakeDeploy(intakeSubsystem))
        .andThen(
            Commands.waitSeconds(0.75)
                .alongWith(Commands.run(() -> {}, intakeSubsystem))
                .withName("Intake Sequence Wait"))
        .withName("Deploy Intake");
  }

  /** Run open-loop toward the retract hall until it trips (startup / homing). */
  public static Command HomeIntake(IntakeSubsystem intakeSubsystem) {
    return seekUntilRetractSwitch(intakeSubsystem).withName("Home Intake Retract");
  }

  public static class HomeIntakeRetract extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public HomeIntakeRetract(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Home Intake Retract");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDeploySpeed(IntakeConstants.HOMING_SPEED);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.isRetractedLimitSwitchTriggered();
    }

    @Override
    public void end(boolean interrupted) {
      intakeSubsystem.zeroIntakeDeploy(true);
      intakeSubsystem.setDesiredDeployPosition(true);
    }
  }

  public static class HomeIntakeDeploy extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public HomeIntakeDeploy(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Home Intake Deploy");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDeploySpeed(-IntakeConstants.HOMING_SPEED);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.isDeployedLimitSwitchTriggered();
    }

    @Override
    public void end(boolean interrupted) {
      intakeSubsystem.zeroIntakeDeploy(false);
      intakeSubsystem.setDesiredDeployPosition(false);
    }
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

  public static class DeployIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Deploy Intake");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDesiredDeployPosition(false);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.isDeployedLimitSwitchTriggered()
          || Math.abs(
                  IntakeConstants.EXTENDED_ANGLE_DEGREES
                      - intakeSubsystem.getCurrentAngle().getDegrees())
              <= IntakeConstants.POSITION_DEADBAND_DEGREES;
    }
  }

  public static class RetractIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Retract Intake");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDesiredDeployPosition(true);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.isRetractedLimitSwitchTriggered()
          || (Math.abs(
                  IntakeConstants.RETRACTED_ANGLE_DEGREES
                      - intakeSubsystem.getCurrentAngle().getDegrees())
              <= IntakeConstants.POSITION_DEADBAND_DEGREES);
    }
  }
}
