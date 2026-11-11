package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveSystemsCheckCommands;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.mech.shooter.ShooterSubsystem;
import frc.robot.util.CommandSimMacXboxController;
import java.util.function.Supplier;

public class SystemCheckBindings {

  private final HoodSubsystem hoodSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final Drive drive;

  private CommandXboxController controller_two = null;

  public SystemCheckBindings(
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
  }

  public void configure() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    if (DriverStation.isJoystickConnected(1)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(1);
        // putting this here because it should only run when we're in sim!

      } else {
        controller_two = new CommandXboxController(1);
      }

      // Right Bumper -- Deploy Intake
      controller_two
          .rightBumper()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.DeployIntake(intakeSubsystem))));

      // Left Bumper -- Retract Intake
      controller_two
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.RetractIntake(intakeSubsystem))));

      controller_two
          .a()
          .onTrue(
              new InstantCommand(
                  () -> {
                    shooterSubsystem.setDesiredRotorVelocity(30);
                    shooterSubsystem.setDesiredTransitionSpeed(ShooterConstants.TRANSITION_SPEED);
                    hopperFloorSubsystem.setDesiredHopperFloorSpeed(
                        HopperFloorConstants.HOPPER_FLOOR_SPEED);
                    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKING_SPEED);
                  }));

      controller_two
          .b()
          .onTrue(
              new HoodCommands.HoodHomingCommand(hoodSubsystem)
                  .andThen(new WaitCommand(1))
                  .andThen(
                      new InstantCommand(
                          () -> hoodSubsystem.setDesiredAngle(HoodConstants.MIN_ANGLE)))
                  .andThen(new WaitCommand(1))
                  .andThen(
                      new InstantCommand(
                          () -> hoodSubsystem.setDesiredAngle(HoodConstants.RETRACTED_POSITION))));

      controller_two
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
                                  intakeSubsystem))));

      controller_two
          .y()
          .onTrue(
              IntakeCommands.HomeIntake(intakeSubsystem)
                  .andThen(new WaitCommand(1))
                  .andThen(IntakeCommands.DeployIntake(intakeSubsystem))
                  .andThen(new WaitCommand(1))
                  .andThen(IntakeCommands.RetractIntake(intakeSubsystem)));

      controller_two
          .leftTrigger()
          .onTrue(
              new InstantCommand(() -> turretSubsystem.homeTurret())
                  .andThen(
                      new InstantCommand(
                          () ->
                              turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(36)))))
                  .andThen(new WaitCommand(1.5))
                  .andThen(
                      new InstantCommand(
                          () ->
                              turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(0))))));
      controller_two
          .rightTrigger()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(DriveSystemsCheckCommands.DriveSystemCheckCommand(drive))));
      controller_two
          .start()
          .onTrue(
              new InstantCommand(
                  () -> {
                    shooterSubsystem.runFarthestShot();
                    hoodSubsystem.setDesiredAngle(HoodConstants.MIN_ANGLE);
                    hopperFloorSubsystem.setDesiredHopperFloorSpeed(
                        HopperFloorConstants.HOPPER_FLOOR_SPEED);
                  }));
    }
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
