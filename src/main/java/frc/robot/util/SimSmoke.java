package frc.robot.util;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/**
 * Sim smoke (./gradlew simSmoke): runs one auto like a match in a windowless simulator, writes a
 * log, and exits. It does nothing in a normal sim run or on the robot.
 */
public class SimSmoke {

  /** Set by build.gradle: the folder to write the log to. Sim smoke runs only when it is set. */
  public static final String LOG_DIR_ENV = "SIM_SMOKE_LOG_DIR";

  /** Set by build.gradle: the auto to run, by its name in the dashboard's auto chooser. */
  public static final String AUTO_ENV = "SIM_SMOKE_AUTO";

  private static final String DEFAULT_AUTO = "B LN-PL";
  // The dashboard writes "selected"; the chooser answers in "active" with the auto it will run.
  private static final String AUTO_CHOOSER = "/SmartDashboard/Auto/PathPlanner Auto/";

  // Counted in robot loops (one loop = 20 ms).
  private static final int BOOT_LOOPS = 50; // 1 s for everything to start
  private static final int CHOOSE_TIMEOUT_LOOPS = 250; // 5 s for the chooser to pick it
  private static final int START_TIMEOUT_LOOPS = 250; // 5 s for the auto to start
  private static final int AUTO_TIMEOUT_LOOPS = 1500; // 30 s for the auto to finish

  /** Where sim smoke is: starting up, choosing the auto, starting it, running it, or done. */
  private enum Phase {
    BOOT,
    CHOOSING,
    STARTING,
    RUNNING,
    DONE
  }

  private final RobotContainer robotContainer;
  private final String autoName;
  private Phase phase = Phase.BOOT;
  private int phaseLoops = 0;
  private Command auto = null;

  /** Builds sim smoke for this robot. Make it only when isRequested() is true. */
  public SimSmoke(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    String requested = System.getenv(AUTO_ENV);
    autoName = requested == null || requested.isBlank() ? DEFAULT_AUTO : requested;
  }

  /** True when this is a sim smoke run: in the simulator, with a log folder set. */
  public static boolean isRequested() {
    return Constants.currentMode == Constants.Mode.SIM && logDir() != null;
  }

  /** The folder to write the log to, or null when this is not a sim smoke run. */
  public static String logDir() {
    return System.getenv(LOG_DIR_ENV);
  }

  /** Runs every loop from Robot.simulationPeriodic(): moves through the phases. */
  public void periodic() {
    phaseLoops++;
    Logger.recordOutput("SimSmoke/Auto", autoName);
    Logger.recordOutput("SimSmoke/Phase", phase.name());
    if (phase == Phase.BOOT && phaseLoops >= BOOT_LOOPS) {
      chooseAuto();
    } else if (phase == Phase.CHOOSING) {
      enableOnceChosen();
    } else if (phase == Phase.STARTING) {
      waitForAutoToStart();
    } else if (phase == Phase.RUNNING) {
      waitForAutoToFinish();
    }
  }

  /** Picks the auto on the pretend dashboard, like a driver does before a match. */
  private void chooseAuto() {
    NetworkTableInstance tables = NetworkTableInstance.getDefault();
    String[] options = tables.getEntry(AUTO_CHOOSER + "options").getStringArray(new String[] {});
    if (!Arrays.asList(options).contains(autoName)) {
      finish(1, "no auto named '" + autoName + "'. The chooser has: " + String.join(", ", options));
      return;
    }
    tables.getEntry(AUTO_CHOOSER + "selected").setString(autoName);
    setPhase(Phase.CHOOSING);
  }

  /** Enables autonomous once the chooser says it will run our auto, not the one chosen before. */
  private void enableOnceChosen() {
    NetworkTableInstance tables = NetworkTableInstance.getDefault();
    String active = tables.getEntry(AUTO_CHOOSER + "active").getString("");
    if (active.equals(autoName)) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      setPhase(Phase.STARTING);
    } else if (phaseLoops > CHOOSE_TIMEOUT_LOOPS) {
      finish(1, "the auto chooser never picked '" + autoName + "'");
    }
  }

  /** Waits until autonomousInit() has scheduled the chosen auto. */
  private void waitForAutoToStart() {
    Command selected = robotContainer.getAutonomousCommand();
    if (DriverStation.isAutonomousEnabled() && selected.isScheduled()) {
      auto = selected;
      setPhase(Phase.RUNNING);
    } else if (phaseLoops > START_TIMEOUT_LOOPS) {
      finish(1, "auto '" + autoName + "' never started");
    }
  }

  /** Waits until the auto ends on its own, or gives up. */
  private void waitForAutoToFinish() {
    if (!auto.isScheduled()) {
      finish(0, "auto '" + autoName + "' finished");
    } else if (phaseLoops > AUTO_TIMEOUT_LOOPS) {
      finish(1, "auto '" + autoName + "' timed out");
    }
  }

  /** Moves to the next phase and restarts its loop count. */
  private void setPhase(Phase next) {
    phase = next;
    phaseLoops = 0;
  }

  /** Disables the robot, closes the log so it is complete, and exits the program. */
  private void finish(int exitCode, String result) {
    phase = Phase.DONE;
    Logger.recordOutput("SimSmoke/Phase", phase.name());
    Logger.recordOutput("SimSmoke/Result", result);
    System.out.println("[SimSmoke] " + result + ", exiting with " + exitCode);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    Logger.end();
    System.exit(exitCode);
  }
}
