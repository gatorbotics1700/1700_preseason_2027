package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.mech.ElevatorCommands;
import org.littletonrobotics.junction.Logger;

/**
 * Headless end-to-end run of the elevator in the full robot sim, used by
 * tools/elevator/run_e2e_sim.sh. Does nothing unless the ELEVATOR_E2E_LOG_DIR environment variable
 * is set in SIM mode.
 *
 * <p>It boots disabled, enables teleop the way a driver station would, runs {@link
 * ElevatorCommands#demoCycle} on the real subsystem, then ends the log and exits: 0 if the demo
 * finished on its own, 1 if it timed out.
 */
public final class SimE2E {
  public static final String LOG_DIR_ENV = "ELEVATOR_E2E_LOG_DIR";
  private static final double BOOT_SECONDS = 1.0;
  private static final double TIMEOUT_SECONDS = 30.0;

  private enum Phase {
    BOOT,
    ENABLING,
    RUNNING,
    DONE
  }

  private static Phase phase = Phase.BOOT;
  private static Command demo;
  private static double phaseStart = 0;

  private SimE2E() {}

  public static String logDir() {
    return System.getenv(LOG_DIR_ENV);
  }

  public static boolean enabled() {
    return Constants.currentMode == Constants.Mode.SIM && logDir() != null;
  }

  /** Call from Robot.simulationPeriodic(). */
  public static void periodic(RobotContainer robotContainer) {
    if (!enabled()) {
      return;
    }
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("E2E/Phase", phase.name());
    switch (phase) {
      case BOOT -> {
        if (now - phaseStart > BOOT_SECONDS) {
          DriverStationSim.setAutonomous(false);
          DriverStationSim.setTest(false);
          DriverStationSim.setEnabled(true);
          DriverStationSim.notifyNewData();
          setPhase(Phase.ENABLING, now);
        }
      }
      case ENABLING -> {
        // teleopInit has already run this loop (it cancels everything), so schedule after it
        if (DriverStation.isTeleopEnabled()) {
          demo = ElevatorCommands.demoCycle(robotContainer.getElevatorSubsystem());
          CommandScheduler.getInstance().schedule(demo);
          setPhase(Phase.RUNNING, now);
        } else if (now - phaseStart > 5.0) {
          finish(1, "robot never enabled");
        }
      }
      case RUNNING -> {
        if (!demo.isScheduled()) {
          finish(0, "demo finished");
        } else if (now - phaseStart > TIMEOUT_SECONDS) {
          finish(1, "demo timed out");
        }
      }
      case DONE -> {}
    }
  }

  private static void setPhase(Phase next, double now) {
    phase = next;
    phaseStart = now;
  }

  private static void finish(int code, String reason) {
    phase = Phase.DONE;
    Logger.recordOutput("E2E/Phase", phase.name());
    Logger.recordOutput("E2E/Result", reason);
    System.out.println("[SimE2E] " + reason + ", exiting with " + code);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    Logger.end();
    System.exit(code);
  }
}
