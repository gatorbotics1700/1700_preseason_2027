package frc.robot.subsystems.mech;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class HubTrackerSubsystem extends SubsystemBase {
  private boolean lastHubState = false;
  private BooleanLogEntry hubActiveLogger;

  public HubTrackerSubsystem() {
    // Initialize the WPILib telemetry data log manager
    DataLog log = DataLogManager.getLog();
    hubActiveLogger = new BooleanLogEntry(log, "/subsystems/HubTracker/isHubActive");

    // Push initial state to SmartDashboard/Elastic Dashboard
    SmartDashboard.putBoolean("Hub Active", false);
  }

  /**
   * Calculates whether the team's Alliance Hub is active. Follows WPILib 2026 Game Rules for
   * REBUILT.
   */
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    // No alliance assigned means we aren't connected or enabled -> Hub inactive
    if (alliance.isEmpty()) {
      return false;
    }

    // Both alliance Hubs are unconditionally active during the 20s Autonomous period
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // If we aren't in Teleop, the hub is inactive
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // Compute active hub state via FMS Game Specific Message
    String gameData = DriverStation.getGameSpecificMessage();

    // Match starting/early Teleop safety fall-through
    if (gameData.isEmpty()) {
      return true;
    }

    double matchTime = DriverStation.getMatchTime();
    boolean redInactiveFirst = gameData.charAt(0) == 'R';
    Alliance myColor = alliance.get();

    // Evaluate shift timings across the 2m 20s Teleop window
    if (matchTime > 105.0) { // Shift 1 (First 35s)
      return redInactiveFirst ? (myColor == Alliance.Blue) : (myColor == Alliance.Red);
    } else if (matchTime > 70.0) { // Shift 2 (Next 35s)
      return redInactiveFirst ? (myColor == Alliance.Red) : (myColor == Alliance.Blue);
    } else if (matchTime > 35.0) { // Shift 3 (Next 35s)
      return redInactiveFirst ? (myColor == Alliance.Blue) : (myColor == Alliance.Red);
    } else { // Shift 4 (Final 35s)
      return redInactiveFirst ? (myColor == Alliance.Red) : (myColor == Alliance.Blue);
    }
  }

  @Override
  public void periodic() {
    boolean currentHubState = isHubActive();

    // Log only on state transitions to keep local data structures clean
    if (currentHubState != lastHubState) {
      hubActiveLogger.append(currentHubState);
      DriverStation.reportWarning("Hub State Changed! Active: " + currentHubState, false);
      lastHubState = currentHubState;
    }

    // Continually feed dashboards (Elastic Dashboard, NetworkTables)
    SmartDashboard.putBoolean("Hub Active", currentHubState);
  }
}
