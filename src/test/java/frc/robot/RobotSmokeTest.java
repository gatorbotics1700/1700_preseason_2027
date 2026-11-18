package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Smoke test: start the whole robot program in the simulator and run it for a few seconds in each
 * mode. It fails if anything crashes, for example a missing config key or a subsystem that throws
 * while it is being built. It does not check that mechanisms do the right thing; other tests do
 * that.
 */
public class RobotSmokeTest {

  // The real robot runs its loop every 20 ms. We step the simulated clock by the same amount.
  private static final double LOOP_SECONDS = 0.02;
  private static final int LOOPS_PER_MODE = 100; // 100 loops x 20 ms = 2 seconds

  private static Robot robot;

  /** Starts the simulated robot once. The robot can only be built once per test run. */
  @BeforeAll
  public static void startRobot() {
    assertTrue(HAL.initialize(500, 0), "The simulator (HAL) did not start");

    // Freeze the simulated clock so it only moves when we step it. This keeps every run the same.
    SimHooks.pauseTiming();

    robot = new Robot();
    robot.robotInit();
  }

  /** The robot runs in Disabled and then in Teleop without crashing. */
  @Test
  public void runsDisabledThenTeleop() {
    setDriverStation(false);
    robot.disabledInit();
    runLoops(false);

    setDriverStation(true);
    robot.teleopInit();
    runLoops(true);
  }

  /** Pretends the Driver Station is connected, enabled or not, and in Teleop. */
  private void setDriverStation(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** Runs the robot's 20 ms loop for two simulated seconds, the way the real robot would. */
  private void runLoops(boolean teleop) {
    for (int i = 0; i < LOOPS_PER_MODE; i++) {
      robot.robotPeriodic();
      if (teleop) {
        robot.teleopPeriodic();
      } else {
        robot.disabledPeriodic();
      }
      robot.simulationPeriodic();
      // "Async" means: move the clock without waiting for the robot's own loop timer to respond.
      // That timer never runs here because we call the loop methods ourselves.
      SimHooks.stepTimingAsync(LOOP_SECONDS);
    }
  }
}
