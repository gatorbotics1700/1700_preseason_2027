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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.PointAtTargetCommand;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommands;
import frc.robot.controls.CodriverBindings;
import frc.robot.controls.CompCodriverBindings;
import frc.robot.controls.CompDriverBindings;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.SysIdBindings;
import frc.robot.controls.SystemCheckBindings;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.mech.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Optional;
import java.util.Set;
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

  // Controllers
  private CommandXboxController controller = null; // port 0
  private CommandXboxController controller_two = null; // port 1

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
    NamedCommands.registerCommand("Home Mechanisms", HomeMechanisms());

    NamedCommands.registerCommand(
        "Point At Hub Command", new PointAtTargetCommand(drive, robotPose));
    NamedCommands.registerCommand("Intaking Command", IntakeCommands.RunIntake(intakeSubsystem));

    NamedCommands.registerCommand(
        "Stop Shooter Command",
        new ShootingCommands.StopShooting(shooterSubsystem, hopperFloorSubsystem));

    NamedCommands.registerCommand(
        "Auto Init", HomeMechanisms().andThen(IntakeCommands.DeployIntake(intakeSubsystem)));

    // Deferred so the closest shot is picked from the pose when the command runs, not at boot
    NamedCommands.registerCommand(
        "Stationary Shot Command",
        Commands.defer(
            () ->
                ShootingCommands.StationaryShootingCommand(
                    shooterSubsystem, hoodSubsystem, hopperFloorSubsystem, drive, robotPose),
            Set.of(drive, shooterSubsystem, hoodSubsystem, hopperFloorSubsystem)));

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

  public void configureDriverButtonBindings() {
    new DriverBindings(
            drive,
            hoodSubsystem,
            hopperFloorSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds)
        .configure();
  }

  public void configureCodriverButtonBindings() {
    new CodriverBindings(
            drive,
            hoodSubsystem,
            hopperFloorSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds)
        .configure();
  }

  public void configureCompDriverButtonBindings() {
    new CompDriverBindings(
            drive,
            hoodSubsystem,
            hopperFloorSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds)
        .configure();
  }

  public void configureCompCodriverButtonBindings() {
    new CompCodriverBindings(
            drive,
            hoodSubsystem,
            hopperFloorSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds)
        .configure();
  }

  public void configureSysIdButtons() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    new SysIdBindings(
            drive,
            hoodSubsystem,
            hopperFloorSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds)
        .configure();
  }

  public void configureSystemCheckButtons() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    new SystemCheckBindings(
            drive,
            hoodSubsystem,
            hopperFloorSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem,
            robotPose,
            chassisSpeeds)
        .configure();
  }

  public void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureCompDriverButtonBindings();
    configureCompCodriverButtonBindings();
    // configureDriverButtonBindings();
    // configureCodriverButtonBindings();
  }

  public void teleopInit() {
    configureButtonBindings();
  }

  public void periodic() {

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

    Command shooterCmd = shooterSubsystem.getCurrentCommand();
    Logger.recordOutput(
        "Commands/ShooterCommand", shooterCmd != null ? shooterCmd.getName() : "None");

    // Logger.recordOutput("Commands/ResetHeading", TODO: make named command an then call it here

    Logger.recordOutput("DriveToFuel/Fuel", vision.getFuelPose(drive.getPose()));
    Logger.recordOutput("Drive/Odometry/Fuel", vision.getFuelPose(drive.getPose()));
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

  public Command HomeMechanisms() {
    return (new HoodCommands.HoodHomingCommand(hoodSubsystem)
            .alongWith(IntakeCommands.HomeIntake(intakeSubsystem)))
        // .alongWith(new InstantCommand(() -> turretSubsystem.homeTurret(), turretSubsystem))
        .withName("Home Mechansims");
  }

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
