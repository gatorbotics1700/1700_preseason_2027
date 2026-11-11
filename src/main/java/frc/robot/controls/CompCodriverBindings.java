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
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.commands.drive.PointAtTargetCommand;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.mech.shooter.ShooterSubsystem;
import frc.robot.util.CommandSimMacXboxController;
import frc.robot.util.shooting.ShotParameters;
import java.util.function.Supplier;

public class CompCodriverBindings {

  private final HoodSubsystem hoodSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final Drive drive;
  private final Supplier<Pose2d> robotPose;

  private CommandXboxController controller_two = null;

  public CompCodriverBindings(
      Drive drive,
      HoodSubsystem hoodSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.hoodSubsystem = hoodSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.drive = drive;
    this.robotPose = robotPose;
  }

  public void configure() {
    if (DriverStation.isJoystickConnected(1)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(1);
        // putting this here because it should only run when we're in sim!

      } else {
        controller_two = new CommandXboxController(1);
      }

      // We want right button to be line up for a shot not shoot on the moe do we have a command for
      // that??
      // TODO Write commands for the following taks and bind em to the indicated buttons
      // Right button - Shot line up have like 3 spots that we go to to shoot
      // X - Vomit outtake TODO: write the vomit command
      //   controller_two
      //   .a()
      //   .onTrue(

      //   )
      /*  controller_two //not in the dahlia edition
      .b()
      .onTrue(
          new InstantCommand(
              () ->
                  CommandScheduler.getInstance()
                      .schedule(IntakeCommands.RetractIntake(intakeSubsystem))));*/

      // Start -- Mech Stop
      controller_two
          .start()
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
                                  intakeSubsystem))));

      // B - stationary shoot
      controller_two
          .b()
          .whileTrue(
              Commands.runOnce(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(
                              ShootingCommands.StationaryShootingCommand(
                                  shooterSubsystem,
                                  hoodSubsystem,
                                  hopperFloorSubsystem,
                                  drive,
                                  robotPose))))
          .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

      // Right trigger - manual shooting
      //   ShootingCommand(
      //     ShooterSubsystem shooterSubsystem,
      //     HoodSubsystem hoodSubsystem,
      //     HopperFloorSubsystem hopperFloorSubsystem,
      //     // TurretSubsystem turretSubsystem,
      //     Supplier<Pose2d> drivetrainPose,
      //     ShotParameters shotParameters)

      controller_two
          .rightTrigger()
          .whileTrue(
              Commands.runOnce(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(
                              new PointAtTargetCommand(drive, robotPose)
                                  .andThen(
                                      new ShootingCommands.ShootingCommand(
                                              shooterSubsystem,
                                              hoodSubsystem,
                                              hopperFloorSubsystem,
                                              robotPose,
                                              new ShotParameters(
                                                  robotPose.get(), HoodConstants.MIN_ANGLE, 50))
                                          .withName("Manual Shooting")))))
          .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

      // Y - stop shooting
      controller_two
          .y()
          .onTrue(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

      // X vomit/outtake
      //   controller_two
      //       .x()
      //       .onTrue(
      //           new InstantCommand(
      //               () ->
      // intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.OUTTAKING_SPEED)));

      // TODO may need to check if we're still in bounds
      // manual hood adjustment - plus 5 degree

      controller_two
          .povUp()
          .onTrue(
              new InstantCommand(
                  () ->
                      hoodSubsystem.setDesiredAngle(
                          new Rotation2d(hoodSubsystem.getCurrentAngle().getDegrees() + 5))));

      // manual hood adjustment - minus 5 degree
      controller_two
          .povDown()
          .onTrue(
              new InstantCommand(
                  () ->
                      hoodSubsystem.setDesiredAngle(
                          new Rotation2d(hoodSubsystem.getCurrentAngle().getDegrees() - 5))));

      /*   // Y -- Turret Angle 0˚
            controller_two
                .y()
                .onTrue(
                    new InstantCommand(
                        () -> turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(0)))));

            // Left Trigger -- Home Turret
            controller_two
                .leftTrigger()
                .onTrue(new InstantCommand(() -> turretSubsystem.homeTurret()).ignoringDisable(true));
      */
      // no turret in the dahlia edition

      // A -- Home Hood

      controller_two
          .a()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(new HoodCommands.HoodHomingCommand(hoodSubsystem))));
    }
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
