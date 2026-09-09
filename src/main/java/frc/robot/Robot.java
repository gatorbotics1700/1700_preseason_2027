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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.Elastic;
import frc.robot.util.RobotConfigLoader;
import frc.robot.util.logging.PDHLogger;
import frc.robot.util.shooting.ShotCalculator;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private ShotCalculator
      shotCalculator; // not actually used anywhere, but we need the constructor to run asap to
  private final PowerDistribution pdh;

  // generate the lookup table

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RoboRIO_Serial", RobotConfigLoader.getSerialNumber());
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs") or to RoboRio
        // ("/home/lvuser/logs")
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    shotCalculator = new ShotCalculator();

    if (Constants.currentMode == Constants.Mode.REPLAY) {
      pdh = null;
    } else {
      pdh = new PowerDistribution(PowerConstants.PDH_CAN_ID, PowerDistribution.ModuleType.kRev);
    }

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // robotContainer.getTurretSubsystem().homeTurret();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    PDHLogger.log(pdh);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.getShooterSubsystem().setDesiredRotorVelocity(0);
    robotContainer.getShooterSubsystem().setDesiredTransitionSpeed(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // In sim, update the robot pose to match the auto chooser selection
    // so the robot appears at the correct starting position before enabling auto
    if (RobotBase.isSimulation()) {
      Optional<Pose2d> startPose = robotContainer.getAutoStartPose();
      if (startPose.isPresent()) {
        robotContainer.getDriveSubsystem().setPose(startPose.get());
      }
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // Set odometry (and sim pose) to the auto start position from the first path
    robotContainer
        .getAutoStartPose()
        .ifPresent(
            startPose -> {
              robotContainer.getDriveSubsystem().setPose(startPose);
            });

    // CommandScheduler.getInstance().schedule(robotContainer.HomeMechanisms());
    // .andThen(IntakeCommands.DeployIntake(robotContainer.getIntakeSubsystem()))
    // .andThen(IntakeCommands.RunIntake(robotContainer.getIntakeSubsystem())));

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    robotContainer.periodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    CommandScheduler.getInstance().cancelAll();
    robotContainer.teleopInit();
    CommandScheduler.getInstance()
        .schedule(
            robotContainer.MechStop(
                robotContainer.getTurretSubsystem(),
                robotContainer.getShooterSubsystem(),
                robotContainer.getHopperFloorSubsystem(),
                robotContainer.getHoodSubsystem(),
                robotContainer.getIntakeSubsystem()));
    // System.out.println("IN TELEOP INIT");
    // CommandScheduler.getInstance()
    //     .schedule(new HoodCommands.HoodHomingCommand(robotContainer.getHoodSubsystem()));
    // CommandScheduler.getInstance()
    //     .schedule(
    //         robotContainer.HomeMechanisms()); // TODO we don't want to actually do this in comps
    Elastic.selectTab("Teleoperated");

    // This makes sure that the autonomous stops running when teleop starts
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotContainer.periodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance()
        .schedule(
            robotContainer.MechStop(
                robotContainer.getTurretSubsystem(),
                robotContainer.getShooterSubsystem(),
                robotContainer.getHopperFloorSubsystem(),
                robotContainer.getHoodSubsystem(),
                robotContainer.getIntakeSubsystem()));
    // robotContainer.configureSysIdButtons();
    robotContainer.configureSystemCheckButtons();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
