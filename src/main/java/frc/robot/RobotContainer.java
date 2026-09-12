// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveOverBumpCommand;
import frc.robot.commands.drive.DriveSystemsCheckCommands;
import frc.robot.commands.drive.DriveUnderTrenchCommand;
import frc.robot.commands.drive.PointAtTargetCommand;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CommandSimMacXboxController;
import frc.robot.util.RobotConfigLoader;
import frc.robot.util.shooting.GamePieceSimulation;
import frc.robot.util.shooting.ShotCalculator;
import frc.robot.util.shooting.ShotParameters;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final HopperFloorSubsystem hopperFloorSubsystem = new HopperFloorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final GamePieceSimulation gamePieceSimulation = new GamePieceSimulation();

  // Controllers
  private CommandXboxController controller = null; // port 0
  private CommandXboxController controller_two = null; // port 3

  // Dashboard inputs
  // private final MultiStepAutoChooser multiStepAutoChooser; // COMMENTED OUT - using PathPlanner
  // pre-made autos
  private final LoggedDashboardChooser<Command> autoChooser;
  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> chassisSpeeds;

  /** Null when {@link Constants.Mode#REPLAY} (no hardware). */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set up robot depending on mode
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_0_NAME, VisionConstants.ROBOT_TO_CAMERA_0),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_1_NAME, VisionConstants.ROBOT_TO_CAMERA_1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_0_NAME,
                    VisionConstants.ROBOT_TO_CAMERA_0,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.ROBOT_TO_CAMERA_1,
                    drive::getPose));
        DriverStation.silenceJoystickConnectionWarning(true);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive);

        break;
    }

    robotPose =
        () -> {
          return drive.getPose();
        };
    chassisSpeeds =
        () -> {
          return drive.getChassisSpeeds();
        };

    // Register named commands for PathPlanner autos
    // NamedCommands.registerCommand("Home Mechanisms", HomeMechanisms());

    NamedCommands.registerCommand(
        "Point At Hub Command", new PointAtTargetCommand(drive, robotPose));
    NamedCommands.registerCommand(
        "Shooter Command",
        new ShootingCommands.ShootOnTheMoveCommand(
            shooterSubsystem,
            hoodSubsystem,
            hopperFloorSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds));
    NamedCommands.registerCommand("Intaking Command", IntakeCommands.RunIntake(intakeSubsystem));
    NamedCommands.registerCommand(
        "Stop Shooter Command",
        new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

    // NamedCommands.registerCommand(
    //     "Auto Init",
    //     HomeMechanisms()
    //         .andThen(IntakeCommands.DeployIntake(intakeSubsystem))
    //         .andThen(IntakeCommands.RunIntake(intakeSubsystem)));

    // Set up auto routines with PathPlanner's auto chooser (using pre-made .auto files)
    autoChooser =
        new LoggedDashboardChooser<>("Auto/PathPlanner Auto", AutoBuilder.buildAutoChooser());

    // COMMENTED OUT - using PathPlanner pre-made autos instead of DynamicAutoBuilder
    // multiStepAutoChooser =
    //     new MultiStepAutoChooser(
    //         intakeSubsystem,
    //         drive,
    //         climberSubsystem,
    //         hoodSubsystem,
    //         shooterSubsystem,
    //         turretSubsystem,
    //         hopperFloorSubsystem,
    //         robotPose,
    //         chassisSpeeds);

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureDriverButtonBindings() {
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

      // A -- Drive Under Trench
      //   controller
      //       .a()
      //       .onTrue(
      //           new InstantCommand(
      //               () -> {
      //                 try {
      //                   CommandScheduler.getInstance()
      //                       .schedule(
      //                           DriveUnderTrenchCommand.driveUnderTrench(drive, shooterSubsystem)
      //                               .withName("DriveUnderTrench"));
      //                 } catch (Exception e) {
      //                   e.printStackTrace();
      //                 }
      //               }));

      //   // Start -- Reset Heading
      //   controller
      //       .start()
      //       .onTrue(
      //           Commands.runOnce(
      //                   () -> {
      //                     if (DriverStation.getAlliance().isPresent()
      //                         && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
      // {
      //                       drive.setPose(
      //                           new Pose2d(
      //                               drive.getPose().getTranslation(),
      //                               new Rotation2d(Math.toRadians(180))));
      //                     } else {
      //                       drive.setPose(
      //                           new Pose2d(
      //                               drive.getPose().getTranslation(),
      //                               new Rotation2d(Math.toRadians(0))));
      //                     }
      //                   },
      //                   drive)
      //               .ignoringDisable(true));

      //   // B -- reverse intake
      //   controller
      //       .b()
      //       .whileTrue(
      //           new InstantCommand(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(IntakeCommands.ReverseIntake(intakeSubsystem))))
      //       .onFalse(
      //           new InstantCommand(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(IntakeCommands.StopIntake(intakeSubsystem))));

      //   // Y -- Drive Over Bump
      //   controller
      //       .y()
      //       .onTrue(
      //           new InstantCommand(
      //               () -> {
      //                 try {
      //                   CommandScheduler.getInstance()
      //                       .schedule(
      //                           DriveOverBumpCommand.driveOverBump(drive, shooterSubsystem)
      //                               .withName("DriveOverBump"));
      //                 } catch (Exception e) {
      //                   e.printStackTrace();
      //                 }
      //               }));

      //   // Back -- Slow Drive Toggle
      //   controller
      //       .back()
      //       .onTrue(
      //           Commands.runOnce(
      //               () -> {
      //                 drive.toggleSlowDrive();
      //               },
      //               drive));

      //   // Left Trigger -- Shoot with Turret (while true)
      //   controller
      //       .leftTrigger()
      //       //   .whileTrue(new InstantCommand(() -> shooterSubsystem.runHardCodedShot()))
      //       //   .onFalse(new InstantCommand(() ->
      // shooterSubsystem.setDesiredTransitionSpeed(0)));
      //       .whileTrue(
      //           Commands.runOnce(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(
      //                           new ShootingCommands.ShootOnTheMoveCommand(
      //                                   shooterSubsystem,
      //                                   hoodSubsystem,
      //                                   hopperFloorSubsystem,
      //                                   turretSubsystem,
      //                                   robotPose,
      //                                   chassisSpeeds)
      //                               .alongWith(new InstantCommand(() ->
      // drive.setSlowDrive(true))))))
      //       .onFalse(
      //           new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem)
      //               .alongWith(new InstantCommand(() -> drive.setSlowDrive(false))));

      //   // X - Point @ Hub & Shoot (without turret) (while true)
      //   controller
      //       .x()
      //       .whileTrue(
      //           Commands.runOnce(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(
      //                           (new PointAtTargetCommand(drive, robotPose))
      //                               .andThen(
      //                                   new ShootingCommands.ShootOnTheMoveCommand(
      //                                       shooterSubsystem,
      //                                       hoodSubsystem,
      //                                       hopperFloorSubsystem,
      //                                       turretSubsystem,
      //                                       robotPose,
      //                                       chassisSpeeds)))))
      //       .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

      //   // Right Trigger -- Run Intake
      //   controller
      //       .rightTrigger()
      //       .whileTrue(
      //           new InstantCommand(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(IntakeCommands.RunIntake(intakeSubsystem))))
      //       .onFalse(
      //           new InstantCommand(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(IntakeCommands.StopIntake(intakeSubsystem))));

      //   // Right Bumper -- Deploy Intake
      //   controller
      //       .rightBumper()
      //       .onTrue(
      //           new InstantCommand(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(IntakeCommands.DeployIntake(intakeSubsystem))));

      //   // Left Bumper -- Retract Intake
      //   controller
      //       .leftBumper()
      //       .onTrue(
      //           new InstantCommand(
      //               () ->
      //                   CommandScheduler.getInstance()
      //                       .schedule(IntakeCommands.RetractIntake(intakeSubsystem))));
    }
  }

  public void configureCodriverButtonBindings() {
    if (DriverStation.isJoystickConnected(1)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(1);
      } else {
        controller_two = new CommandXboxController(1);
      }

      if (Constants.currentMode == Constants.Mode.SIM) {
        controller_two
            .a()
            .onTrue(
                new InstantCommand(
                    () -> {
                      // Use current pose and chassis speeds at this instant so all values match.
                      Pose2d pose = drive.getPose();

                      ChassisSpeeds cs = drive.getChassisSpeeds();
                      ShotParameters params =
                          ShotCalculator.calculateShot(pose, cs, FieldCoordinates.BLUE_HUB);

                      gamePieceSimulation.launchFuelBall(
                          ShotCalculator.getFieldToShooter(pose, ShooterConstants.BOT_TO_SHOOTER),
                          cs,
                          drive.getRotation(),
                          params.shotSpeed,
                          params.turretAngle,
                          params.hoodAngle);
                    }));
        controller_two
            .b()
            .onTrue(
                AutoBuilder.pathfindToPose(
                        new Pose2d(1, FieldCoordinates.BLUE_HUB.getY(), new Rotation2d()),
                        new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
                    .andThen(
                        Commands.parallel(
                            AutoBuilder.pathfindToPose(
                                new Pose2d(3, FieldCoordinates.BLUE_HUB.getY(), new Rotation2d()),
                                new PathConstraints(
                                    0.75, 12, Math.toRadians(700), Math.toRadians(1000))),
                            Commands.waitSeconds(0.2)
                                .andThen(
                                    Commands.runOnce(
                                        () -> {
                                          // Use current pose and chassis speeds at this instant so
                                          // all values match.
                                          Pose2d pose = drive.getPose();

                                          ChassisSpeeds cs = drive.getChassisSpeeds();
                                          ShotParameters params =
                                              ShotCalculator.calculateShot(
                                                  pose, cs, FieldCoordinates.BLUE_HUB);

                                          gamePieceSimulation.launchFuelBall(
                                              ShotCalculator.getFieldToShooter(
                                                  pose, ShooterConstants.BOT_TO_SHOOTER),
                                              cs,
                                              pose.getRotation(),
                                              params.shotSpeed,
                                              params.turretAngle,
                                              params.hoodAngle);
                                        })))));
      } else {
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
        controller_two
            .leftTrigger()
            .whileTrue(
                Commands.runOnce(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(
                                new ShootingCommands.ShootOnTheMoveCommand(
                                    shooterSubsystem,
                                    hoodSubsystem,
                                    hopperFloorSubsystem,
                                    turretSubsystem,
                                    robotPose,
                                    chassisSpeeds))))
            .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

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

  public void configureCompDriverButtonBindings() {
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

      // A -- Drive Under Trench
      controller
          .a()
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

      // B -- Reset Heading
      controller
          .b()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        System.out.println("Reset Heading Started");
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
                        System.out.println(
                            "Robot odometry angle: " + drive.getRotation().getDegrees());
                      },
                      drive)
                  .ignoringDisable(true));

      // X -- Drive Over Bump
      controller
          .x()
          .onTrue(
              new InstantCommand(
                  () -> {
                    try {
                      CommandScheduler.getInstance()
                          .schedule(
                              DriveOverBumpCommand.driveOverBump(drive, shooterSubsystem)
                                  .withName("DriveOverBump"));
                    } catch (Exception e) {
                      e.printStackTrace();
                    }
                  }));

      // Y -- Slow Drive Toggle
      controller
          .y()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    drive.toggleSlowDrive();
                  },
                  drive));

      // Left Trigger -- Shoot with Turret (while true)
      controller
          .leftTrigger()
          .whileTrue(
              Commands.runOnce(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(
                              new ShootingCommands.ShootOnTheMoveCommand(
                                  shooterSubsystem,
                                  hoodSubsystem,
                                  hopperFloorSubsystem,
                                  turretSubsystem,
                                  robotPose,
                                  chassisSpeeds))))
          .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

      // Left Bumper -- Point @ Hub & Shoot (without turret) (while true)
      controller
          .leftBumper()
          .whileTrue(
              Commands.runOnce(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(
                              (new PointAtTargetCommand(drive, robotPose))
                                  .andThen(
                                      new ShootingCommands.ShootOnTheMoveCommand(
                                          shooterSubsystem,
                                          hoodSubsystem,
                                          hopperFloorSubsystem,
                                          turretSubsystem,
                                          robotPose,
                                          chassisSpeeds)))))
          .onFalse(new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

      // Right Trigger -- Run Intake
      controller
          .rightTrigger()
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

  public void configureCompCodriverButtonBindings() {
    // TODO actually call this method
    if (DriverStation.isJoystickConnected(1)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(1);
        // putting this here because it should only run when we're in sim!

      } else {
        controller_two = new CommandXboxController(1);
      }

      // B -- Retract Intake
      controller_two
          .b()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(IntakeCommands.RetractIntake(intakeSubsystem))));

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

      // Y -- Turret Angle 0˚
      controller_two
          .y()
          .onTrue(
              new InstantCommand(
                  () -> turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(0)))));

      // Left Trigger -- Home Turret
      controller_two
          .leftTrigger()
          .onTrue(new InstantCommand(() -> turretSubsystem.homeTurret()).ignoringDisable(true));

      // Left Bumper -- Home Hood
      controller_two
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(new HoodCommands.HoodHomingCommand(hoodSubsystem))));
    }
  }

  public void configureSysIdButtons() {
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

  public Command getAutonomousCommand() {
    // Using PathPlanner pre-made autos
    Command selected = autoChooser.get();
    return selected != null ? selected : Commands.none();

    // COMMENTED OUT - using PathPlanner pre-made autos instead of DynamicAutoBuilder
    // try {
    //   return multiStepAutoChooser.getAutonomousCommand();
    // } catch (Exception ioe) {
    //   System.out.println("bad io error");
    // return Commands.none();
    // }
  }

  public Optional<Pose2d> getAutoStartPose() {
    // TODO: Implement start pose extraction from PathPlanner auto if needed
    return Optional.empty();

    // COMMENTED OUT - using PathPlanner pre-made autos instead of DynamicAutoBuilder
    // return multiStepAutoChooser.getAutoStartPose();
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  private double deadband(double value, double deadband) {
    // If controller reads very tiny value close to zero, we don't want to make the
    // robot think it has to move
    // Without deadband, robot will think it has to move, and then it will go crazy
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0; // Return 0 if absolute value is within desired margin of error
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

  public void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // configureCompDriverButtonBindings();
    // configureCompCodriverButtonBindings(); // TODO: IMPORTANT SWITCH THIS BEFORE MATCHES
    configureDriverButtonBindings();
    // configureCodriverButtonBindings();
  }

  public void configureSystemCheckButtons() {
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

  public void teleopInit() {
    configureButtonBindings();
  }

  public void periodic() {
    if (Constants.currentMode == Constants.simMode) {
      gamePieceSimulation.updateBalls();
    }

    robotContainerLogs();

    // multiStepAutoChooser.updateChooserOptions();

    // Print path name to console me thinks
    // String selectedPathName = multiStepAutoChooser.getSelectedPathName();
    // System.out.flush(); // Ensure output appears immediately
  }

  public void robotContainerLogs() {
    // Log command scheduler status
    Logger.recordOutput("Commands/SchedulerActive", true);
    Logger.recordOutput("Commands/LogTime", System.currentTimeMillis());

    // Log command information with names
    Command driveCmd = drive.getCurrentCommand();

    Logger.recordOutput("Commands/DriveCommand", driveCmd != null ? driveCmd.getName() : "None");

    // Log if commands are running
    Logger.recordOutput("Commands/DriveCommandActive", driveCmd != null);
    Logger.recordOutput(
        "Commands/DriveToFuelActive",
        driveCmd != null ? driveCmd.getName().equals("DriveToFuel") : false);

    Logger.recordOutput("DriveToFuel/Fuel", vision.getFuelPose(drive.getPose()));
    Logger.recordOutput("Drive/Odometry/Fuel", vision.getFuelPose(drive.getPose()));

    Logger.recordOutput("Mech/Valid Shot", getValidShot());
  }

  public boolean getValidShot() {
    Translation3d target;

    if (FieldCoordinates.BLUE_BUMP_AND_TRENCH_X <= drive.getPose().getX()
        && drive.getPose().getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
      if (FieldCoordinates.FIELD_CENTER.getY() < drive.getPose().getY()) {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_RIGHT_FUNNELING
                : FieldCoordinates.RED_LEFT_FUNNELING;

      } else {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_LEFT_FUNNELING
                : FieldCoordinates.RED_RIGHT_FUNNELING;
      }

    } else {
      target =
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldCoordinates.BLUE_HUB
              : FieldCoordinates.RED_HUB;
    }

    ShotParameters params =
        ShotCalculator.calculateShot(drive.getPose(), chassisSpeeds.get(), target);

    if (params.shotSpeed == 0) {
      return false;
    }
    return true;
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

  //   public Command HomeMechanisms() {
  //     return (new HoodCommands.HoodHomingCommand(hoodSubsystem)
  //             .alongWith(IntakeCommands.HomeIntake(intakeSubsystem)))
  //         // .alongWith(new InstantCommand(() -> turretSubsystem.homeTurret(), turretSubsystem))
  //         .withName("Home Mechansims");
  //   }

  public Command TestShot(ShooterSubsystem shooterSubsystem) {
    return (new InstantCommand(
            () -> {
              shooterSubsystem.setDesiredRotorVelocity(40);
              shooterSubsystem.setDesiredTransitionSpeed(ShooterConstants.TRANSITION_SPEED);
            },
            shooterSubsystem)
        .withName("TestShot"));
  }

  public TurretSubsystem getTurretSubsystem() {
    return turretSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public HoodSubsystem getHoodSubsystem() {
    return hoodSubsystem;
  }

  public HopperFloorSubsystem getHopperFloorSubsystem() {
    return hopperFloorSubsystem;
  }
}
