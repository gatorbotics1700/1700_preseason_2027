package frc.robot.diagnostics;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.diagnostics.LogFile.NumberSample;
import frc.robot.diagnostics.PositionChecks.Move;
import frc.robot.util.SimSmoke;
import frc.robot.view3d.ElevatorModel;
import frc.robot.view3d.View3dChecks;
import java.util.ArrayList;
import java.util.List;

/**
 * Things to look for in the elevator's log: the standard position checks, set up from
 * ElevatorConstants, plus a few of its own.
 */
public final class ElevatorChecks {

  // Where the elevator logs its hardware readings, and what its code decided.
  static final String IO = "/Mech/Elevator/IO/";
  static final String OUT = "/RealOutputs/Mech/Elevator/";

  private ElevatorChecks() {}

  /** Every elevator check. */
  public static List<LogCheck> all() {
    List<LogCheck> checks = new ArrayList<>();
    checks.addAll(PositionChecks.all(settings()));
    checks.add(new SimHeightMatchesEncoder());
    checks.add(new DemoVisitsEveryHeight());
    checks.add(new View3dChecks.LayoutKeysAreLogged(ElevatorModel.LAYOUT));
    return checks;
  }

  /** The elevator's log keys and limits, for the standard position checks. */
  static PositionChecks.Settings settings() {
    PositionChecks.Settings elevator = new PositionChecks.Settings();
    elevator.name = "Elevator";
    elevator.unit = "m";
    elevator.positionKey = IO + "PositionMeters";
    elevator.goalKey = OUT + "GoalMeters";
    elevator.modeKey = OUT + "ControlMode";
    elevator.homedKey = OUT + "Homed";
    elevator.homeSwitchKey = IO + "BottomLimitPressed";
    elevator.voltsKey = IO + "AppliedVolts";
    elevator.currentKey = IO + "StatorCurrentAmps";
    elevator.homePosition = ElevatorConstants.MIN_HEIGHT_METERS;
    elevator.minPosition = ElevatorConstants.MIN_HEIGHT_METERS;
    elevator.maxPosition = ElevatorConstants.MAX_HEIGHT_METERS;
    elevator.tolerance = ElevatorConstants.POSITION_TOLERANCE_METERS;
    elevator.maxOvershoot = 0.02;
    elevator.travelSlack = 0.005;
    elevator.cruiseVelocity = ElevatorConstants.CRUISE_VELOCITY_MPS;
    elevator.acceleration = ElevatorConstants.ACCELERATION_MPS2;
    elevator.jerk = ElevatorConstants.JERK_MPS3;
    elevator.statorCurrentLimitAmps = ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS;
    return elevator;
  }

  /** Simulator only: once homed, the encoder agrees with the physics' true height when still. */
  static class SimHeightMatchesEncoder implements LogCheck {
    private static final double MAX_ERROR_METERS = 0.002;
    // Slower than this counts as still. While moving, the encoder trails by one reading.
    private static final double STILL_METERS_PER_SEC = 0.05;
    private static final double AFTER_HOMING_SECONDS = 0.05;
    // The top hard stop sits a little above the top of the travel (plus rounding).
    private static final double TOP_HARD_STOP_SLACK_METERS = 0.02 + 1e-6;

    /** The check's name in the report. */
    public String name() {
      return "Elevator (sim): encoder matches the true height";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {
        OUT + "Sim/TrueHeightMeters",
        IO + "PositionMeters",
        IO + "VelocityMetersPerSec",
        OUT + "Homed"
      };
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      double homedTime = PositionChecks.firstTrue(log.booleans(OUT + "Homed"));
      double worstError = 0;
      double topStop = ElevatorConstants.MAX_HEIGHT_METERS + TOP_HARD_STOP_SLACK_METERS;
      for (NumberSample truth : log.numbers(OUT + "Sim/TrueHeightMeters")) {
        if (truth.value > topStop) {
          report.error(
              String.format(
                  "hit the top hard stop (%.3f m) at %.2f s", truth.value, truth.timeSeconds));
        }
        double speed = Math.abs(log.numberAt(IO + "VelocityMetersPerSec", truth.timeSeconds));
        if (truth.timeSeconds > homedTime + AFTER_HOMING_SECONDS && speed < STILL_METERS_PER_SEC) {
          double encoder = log.numberAt(IO + "PositionMeters", truth.timeSeconds);
          worstError = Math.max(worstError, Math.abs(encoder - truth.value));
        }
      }
      if (worstError > MAX_ERROR_METERS) {
        report.error(String.format("encoder off the true height by %.1f mm", worstError * 1000));
      }
    }
  }

  /** Simulator only: the Elevator Demo auto visits stow, low, high, mid, then stow. */
  static class DemoVisitsEveryHeight implements LogCheck {
    // Goals closer than this count as the same height.
    private static final double SAME_HEIGHT_METERS = 1e-6;

    /** The check's name in the report. */
    public String name() {
      return "Elevator (sim): demo visits every height";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {SimSmoke.AUTO_LOG_KEY, OUT + "GoalMeters", OUT + "ControlMode"};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      if (!log.texts(SimSmoke.AUTO_LOG_KEY).get(0).value.equals("Elevator Demo")) {
        report.note("not the Elevator Demo auto");
        return;
      }
      double[] wanted = {
        ElevatorConstants.STOW_HEIGHT_METERS,
        ElevatorConstants.LOW_HEIGHT_METERS,
        ElevatorConstants.HIGH_HEIGHT_METERS,
        ElevatorConstants.MID_HEIGHT_METERS,
        ElevatorConstants.STOW_HEIGHT_METERS
      };
      List<Move> moves = PositionChecks.findMoves(log, settings());
      if (!visitedInOrder(moves, wanted)) {
        report.error("goals were" + goalList(moves) + ", expected stow, low, high, mid, stow");
      }
    }

    /** True if the moves' goals are exactly these heights, in this order. */
    private static boolean visitedInOrder(List<Move> moves, double[] wanted) {
      if (moves.size() != wanted.length) {
        return false;
      }
      for (int i = 0; i < wanted.length; i++) {
        if (Math.abs(moves.get(i).goal - wanted[i]) > SAME_HEIGHT_METERS) {
          return false;
        }
      }
      return true;
    }

    /** The moves' goals as text, for the report. */
    private static String goalList(List<Move> moves) {
      StringBuilder goals = new StringBuilder();
      for (Move move : moves) {
        goals.append(String.format(" %.2f", move.goal));
      }
      return goals.toString();
    }
  }
}
