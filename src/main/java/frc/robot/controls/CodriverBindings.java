package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.CommandSimMacXboxController;
import java.util.function.Supplier;

public class CodriverBindings {

  private final HoodSubsystem hoodSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;

  private CommandXboxController controller_two = null;

  public CodriverBindings(
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
  }

  public void configure() {

    if (DriverStation.isJoystickConnected(1)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(1);
      } else {
        controller_two = new CommandXboxController(1);
      }

      if (Constants.currentMode == Constants.Mode.REAL) {
        // A -- Retract Intake
        controller_two
            .a()
            .onTrue(
                new InstantCommand(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(IntakeCommands.RetractIntake(intakeSubsystem))));

        // B -- Deploy Intake
        controller_two
            .b()
            .onTrue(
                new InstantCommand(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(IntakeCommands.DeployIntake(intakeSubsystem))));

        // X -- Mech Stop
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

        // Y -- Hood Home
        controller_two.y().onTrue(new HoodCommands.HoodHomingCommand(hoodSubsystem));

        controller_two
            .leftBumper()
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

        // Left Trigger -- Shoot with Turret (while true)
        // controller_two
        //     .leftTrigger()
        //     .whileTrue(
        //         Commands.runOnce(
        //             () ->
        //                 CommandScheduler.getInstance()
        //                     .schedule(
        //                         new ShootingCommands.ShootOnTheMoveCommand(
        //                             shooterSubsystem,
        //                             hoodSubsystem,
        //                             hopperFloorSubsystem,
        //                             turretSubsystem,
        //                             robotPose,
        //                             chassisSpeeds))))
        //     .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

        // Right Bumper -- Home Hood
        controller_two
            .rightBumper()
            .onTrue(
                new InstantCommand(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(new HoodCommands.HoodHomingCommand(hoodSubsystem))));
      }
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
