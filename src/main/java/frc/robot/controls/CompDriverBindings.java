package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveUnderTrenchCommand;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.CommandSimMacXboxController;
import frc.robot.util.RobotConfigLoader;
import java.util.function.Supplier;

public class CompDriverBindings {

  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Drive drive;

  private CommandXboxController controller = null;

  public CompDriverBindings(
      Drive drive,
      HoodSubsystem hoodSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
  }

  public void configure() {
    if (DriverStation.isJoystickConnected(0)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller = new CommandSimMacXboxController(0);
      } else {
        controller = new CommandXboxController(0);
      }

      // Default command, normal field-relative drive
      Trigger driverControl =
          new Trigger(
              () ->
                  Math.abs(controller.getLeftY()) > 0.1
                      || Math.abs(controller.getLeftX()) > 0.1
                      || Math.abs(controller.getRightX()) > 0.1);

      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        driverControl
            .whileTrue(
                DriveCommands.joystickDrive(
                    drive,
                    () -> modifyJoystickAxis(controller.getLeftY()), // Changed to raw values
                    () -> modifyJoystickAxis(controller.getLeftX()), // Changed to raw values
                    () -> modifyJoystickAxis(-controller.getRightX()))) // Changed to raw values
            .onFalse(DriveCommands.stopDriveCommand(drive));
      } else {
        driverControl
            .whileTrue(
                DriveCommands.joystickDrive(
                    drive,
                    () -> modifyJoystickAxis(-controller.getLeftY()), // Changed to raw values
                    () -> modifyJoystickAxis(-controller.getLeftX()), // Changed to raw values
                    () -> modifyJoystickAxis(-controller.getRightX()))) // Changed to raw values
            .onFalse(DriveCommands.stopDriveCommand(drive));
      }

      //  pov down -- Drive Under Trench
      controller
          .povDown()
          .onTrue(
              new InstantCommand(
                  () -> {
                    try {
                      CommandScheduler.getInstance()
                          .schedule(
                              DriveUnderTrenchCommand.driveUnderTrench(drive, shooterSubsystem)
                                  .withName("DriveUnderTrench"));
                    } catch (Exception e) {
                      e.printStackTrace();
                    }
                  }));
      controller
          .y()
          .whileTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(runAgitateTransition(shooterSubsystem, intakeSubsystem))))
          .onFalse(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(stopAgitateTransition(shooterSubsystem, intakeSubsystem))));
      // START-- Reset Heading
      controller
          .start()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        if (DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                          drive.setPose(
                              new Pose2d(
                                  drive.getPose().getTranslation(),
                                  new Rotation2d(Math.toRadians(180))));
                        } else {
                          drive.setPose(
                              new Pose2d(
                                  drive.getPose().getTranslation(),
                                  new Rotation2d(Math.toRadians(0))));
                        }
                      },
                      drive)
                  .ignoringDisable(true));

      controller
          .x()
          .whileTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.ReverseIntake(intakeSubsystem))))
          .onFalse(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.StopIntake(intakeSubsystem))));

      /*    controller
      .x()
      .onTrue(
          new InstantCommand(
              () ->
                  CommandScheduler.getInstance()
                      .schedule(
                          MechStop(
                              turretSubsystem,
                              shooterSubsystem,
                              hopperFloorSubsystem,
                              hoodSubsystem,
                              intakeSubsystem)))); */

      // X -- Slow Drive Toggle
      /*   controller
      .x()
      .onTrue(
          Commands.runOnce(
              () -> {
                drive.toggleSlowDrive();
              },
              drive));
              */

      // B -- Run Intake Hold
      controller
          .b()
          .whileTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.RunIntake(intakeSubsystem))))
          .onFalse(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.StopIntake(intakeSubsystem))));

      // Right Bumper -- Deploy / Retract Intake Toggle
      controller
          .rightBumper()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.ToggleIntake(intakeSubsystem))));
    }
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyJoystickAxis(double value) {
    // Deadband
    value = deadband(value, 0.025);

    // Square the axis
    value =
        Math.copySign(
            RobotConfigLoader.getDouble("container.joystick_scale_factor") * Math.pow(value, 2),
            value);

    if (drive.getSlowDrive()) {
      return 0.5 * value;
    }

    return value;
  }

  public Command runAgitateTransition(
      ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    return (new InstantCommand(
        () -> {
          shooterSubsystem.setDesiredTransitionSpeed(-0.1);
        },
        shooterSubsystem));
  }

  public Command stopAgitateTransition(
      ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    return (new InstantCommand(
        () -> {
          shooterSubsystem.setDesiredTransitionSpeed(0);
        },
        shooterSubsystem));
  }

  public Command MechStop(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      HoodSubsystem hoodSubsystem,
      IntakeSubsystem intakeSubsystem) {
    return (new InstantCommand(
                () -> {
                  turretSubsystem.setDesiredAngle(turretSubsystem.getCurrentAngle());
                  shooterSubsystem.setDesiredRotorVelocity(0);
                  hopperFloorSubsystem.setDesiredHopperFloorSpeed(0);
                  shooterSubsystem.setDesiredTransitionSpeed(0);
                  hoodSubsystem.setDesiredAngle(hoodSubsystem.getCurrentAngle());
                  hoodSubsystem.setHoodSpeed(0);
                },
                turretSubsystem,
                shooterSubsystem,
                hopperFloorSubsystem,
                hoodSubsystem,
                intakeSubsystem)
            .alongWith(IntakeCommands.StopIntake(intakeSubsystem)))
        .withName("Mech Stop");
  }
}
