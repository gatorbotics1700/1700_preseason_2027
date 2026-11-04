package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.CommandSimMacXboxController;
import java.util.function.Supplier;

public class SysIdBindings {

  private final Drive drive;

  private CommandXboxController controller_two = null;

  public SysIdBindings(
      Drive drive,
      HoodSubsystem hoodSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
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

      // TODO: drivetrain sysid buttons -- uncomment for use
      controller_two.a().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      controller_two.b().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      controller_two.x().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      controller_two.y().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

      // TODO: hood sysid buttons -- uncomment for use
      // controller_two.x().whileTrue(hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // controller_two.y().whileTrue(hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // controller_two.a().whileTrue(hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // controller_two.b().whileTrue(hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

      // controller_two.rightTrigger().onTrue(new InstantCommand(() -> hoodSubsystem.zeroHood()));

      // TODO: intake sysid buttons -- uncomment for use
      // controller_two.x().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // controller_two.y().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // controller_two
      //     .a()
      //     .whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // controller_two
      //     .b()
      //     .whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

      // controller_two
      //     .rightBumper()
      //     .onTrue(
      //         new InstantCommand(() ->
      // intakeSubsystem.zeroIntakeDeploy()).ignoringDisable(true));

      // TODO: shooter sysid buttons -- uncomment for use
      // controller_two.x().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // controller_two.y().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // controller_two
      //     .a()
      //     .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // controller_two
      //     .b()
      //     .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

      // TODO: turret sysid buttons -- uncomment for use
      // controller_two.x().whileTrue(turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // controller_two.y().whileTrue(turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // controller_two
      //     .a()
      //     .whileTrue(turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // controller_two
      //     .b()
      //     .whileTrue(turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

      // controller_two
      //     .rightBumper()
      //     .onTrue(new InstantCommand(() -> turretSubsystem.homeTurret()).ignoringDisable(true));
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
