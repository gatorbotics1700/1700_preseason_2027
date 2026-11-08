package frc.robot.diagnostics;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.diagnostics.LogFile.BooleanSample;
import frc.robot.diagnostics.LogFile.NumberSample;
import frc.robot.diagnostics.LogFile.TextSample;
import java.util.ArrayList;
import java.util.List;

/**
 * Things to look for in the elevator's log, with limits read from ElevatorConstants. To add a
 * check, copy StaysInVoltageAndCurrentLimits and add it to all().
 */
public final class ElevatorChecks {

  // Where the elevator logs its hardware readings, and what its code decided.
  static final String IO = "/Mech/Elevator/IO/";
  static final String OUT = "/RealOutputs/Mech/Elevator/";

  private ElevatorChecks() {}

  /** Every elevator check. */
  public static List<LogCheck> all() {
    List<LogCheck> checks = new ArrayList<>();
    checks.add(new HomesBeforeMoving());
    checks.add(new MovesReachGoalInTime());
    checks.add(new StaysInTravel());
    checks.add(new StaysInVoltageAndCurrentLimits());
    checks.add(new SimHeightMatchesEncoder());
    checks.add(new DemoVisitsEveryHeight());
    return checks;
  }

  /** The bottom switch closes and Homed turns on before any raised goal is set. */
  static class HomesBeforeMoving implements LogCheck {
    public String name() {
      return "Elevator: homes before moving";
    }

    public String[] requiredKeys() {
      return new String[] {IO + "BottomLimitPressed", OUT + "Homed", OUT + "GoalMeters"};
    }

    public void check(LogFile log, Report report) {
      double homedTime = firstTrue(log.booleans(OUT + "Homed"));
      double switchTime = firstTrue(log.booleans(IO + "BottomLimitPressed"));
      for (NumberSample goal : log.numbers(OUT + "GoalMeters")) {
        boolean raised = goal.value > ElevatorConstants.POSITION_TOLERANCE_METERS;
        if (raised && !(goal.timeSeconds >= homedTime)) {
          report.error(
              String.format("goal %.2f m at %.2f s, before homing", goal.value, goal.timeSeconds));
        }
      }
      if (Double.isNaN(homedTime)) {
        report.note("never homed");
      } else if (Double.isNaN(switchTime) || switchTime > homedTime) {
        report.error(String.format("homed at %.2f s without the bottom switch closing", homedTime));
      } else {
        report.note(String.format("homed at %.2f s", homedTime));
      }
    }
  }

  /** Each move settles within tolerance in about its profile time, without overshoot. */
  static class MovesReachGoalInTime implements LogCheck {
    // Extra time allowed on top of the profile, for the last bit of settling.
    private static final double SETTLE_MARGIN_SECONDS = 0.6;
    private static final double MAX_OVERSHOOT_METERS = 0.02;

    public String name() {
      return "Elevator: moves reach the goal in time";
    }

    public String[] requiredKeys() {
      return new String[] {IO + "PositionMeters", OUT + "GoalMeters", OUT + "ControlMode"};
    }

    public void check(LogFile log, Report report) {
      for (Move move : findMoves(log)) {
        checkMove(log, move, report);
      }
    }

    /** Checks one move: settled, how long it took, and how far past the goal it went. */
    private void checkMove(LogFile log, Move move, Report report) {
      double startMeters = log.numberAt(IO + "PositionMeters", move.startSeconds);
      double allowedSeconds =
          profileSeconds(Math.abs(move.goalMeters - startMeters)) + SETTLE_MARGIN_SECONDS;
      double settledSeconds = settledTime(log, move);
      String what =
          String.format(
              "move %.2f -> %.2f m at %.2f s", startMeters, move.goalMeters, move.startSeconds);

      if (Double.isNaN(settledSeconds)) {
        if (move.endSeconds - move.startSeconds >= allowedSeconds) {
          report.error(what + ": never reached the goal");
        } else {
          report.note(what + ": interrupted before it was due");
        }
        return;
      }
      double tookSeconds = settledSeconds - move.startSeconds;
      String timing =
          String.format(
              "%s: settled in %.2f s (allowed %.2f s)", what, tookSeconds, allowedSeconds);
      if (tookSeconds > allowedSeconds) {
        report.error(timing);
      } else {
        report.note(timing);
      }
      double overshootMeters = overshoot(log, move, startMeters);
      if (overshootMeters > MAX_OVERSHOOT_METERS) {
        report.error(String.format("%s: went %.1f cm past the goal", what, overshootMeters * 100));
      }
    }

    /** The time after which the carriage stays within tolerance until the move ends, or NaN. */
    private static double settledTime(LogFile log, Move move) {
      // Heights are logged only when they change, so start from the height at the move's start.
      double startMeters = log.numberAt(IO + "PositionMeters", move.startSeconds);
      double tolerance = ElevatorConstants.POSITION_TOLERANCE_METERS;
      double settled =
          Math.abs(startMeters - move.goalMeters) <= tolerance ? move.startSeconds : Double.NaN;
      for (NumberSample sample : log.numbers(IO + "PositionMeters")) {
        if (sample.timeSeconds < move.startSeconds || sample.timeSeconds > move.endSeconds) {
          continue;
        }
        boolean inTolerance =
            Math.abs(sample.value - move.goalMeters) <= ElevatorConstants.POSITION_TOLERANCE_METERS;
        if (!inTolerance) {
          settled = Double.NaN;
        } else if (Double.isNaN(settled)) {
          settled = sample.timeSeconds;
        }
      }
      return settled;
    }

    /** How far past the goal the carriage went during the move, in its direction of travel. */
    private static double overshoot(LogFile log, Move move, double startMeters) {
      boolean goingUp = move.goalMeters >= startMeters;
      double worst = 0;
      for (NumberSample sample : log.numbers(IO + "PositionMeters")) {
        if (sample.timeSeconds >= move.startSeconds && sample.timeSeconds <= move.endSeconds) {
          double past = goingUp ? sample.value - move.goalMeters : move.goalMeters - sample.value;
          worst = Math.max(worst, past);
        }
      }
      return worst;
    }
  }

  /** After homing, the carriage height stays inside the travel limits. */
  static class StaysInTravel implements LogCheck {
    // Allowed past the limits, for sensor noise.
    private static final double SLACK_METERS = 0.005;
    // The position reset lands one loop after the switch closes, so skip a little after homing.
    private static final double AFTER_HOMING_SECONDS = 0.03;

    public String name() {
      return "Elevator: stays inside its travel";
    }

    public String[] requiredKeys() {
      return new String[] {IO + "PositionMeters", OUT + "Homed"};
    }

    public void check(LogFile log, Report report) {
      double homedTime = firstTrue(log.booleans(OUT + "Homed"));
      double lowest = ElevatorConstants.MIN_HEIGHT_METERS - SLACK_METERS;
      double highest = ElevatorConstants.MAX_HEIGHT_METERS + SLACK_METERS;
      for (NumberSample sample : log.numbers(IO + "PositionMeters")) {
        boolean afterHoming = sample.timeSeconds > homedTime + AFTER_HOMING_SECONDS;
        if (afterHoming && (sample.value < lowest || sample.value > highest)) {
          report.error(String.format("height %.3f m at %.2f s", sample.value, sample.timeSeconds));
          return;
        }
      }
    }
  }

  /** Voltage stays within the battery's range and current within the motor's limit. */
  static class StaysInVoltageAndCurrentLimits implements LogCheck {
    private static final double MAX_VOLTS = 12.5;
    // The motor's current limit is not exact; allow a little over it.
    private static final double CURRENT_SLACK_AMPS = 5.0;

    public String name() {
      return "Elevator: stays inside voltage and current limits";
    }

    public String[] requiredKeys() {
      return new String[] {IO + "AppliedVolts", IO + "StatorCurrentAmps"};
    }

    public void check(LogFile log, Report report) {
      double maxVolts = largest(log.numbers(IO + "AppliedVolts"), true);
      double maxAmps = largest(log.numbers(IO + "StatorCurrentAmps"), false);
      double ampLimit = ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS + CURRENT_SLACK_AMPS;
      if (maxVolts > MAX_VOLTS) {
        report.error(String.format("applied %.2f V", maxVolts));
      }
      if (maxAmps > ampLimit) {
        report.error(
            String.format("stator current reached %.1f A (limit %.0f A)", maxAmps, ampLimit));
      }
      report.note(String.format("peak current %.1f A", maxAmps));
    }
  }

  /** Simulator only: once homed, the encoder agrees with the physics' true height when still. */
  static class SimHeightMatchesEncoder implements LogCheck {
    private static final double MAX_ERROR_METERS = 0.002;
    // Slower than this counts as still. While moving, the encoder trails by one reading.
    private static final double STILL_METERS_PER_SEC = 0.05;
    private static final double AFTER_HOMING_SECONDS = 0.05;
    private static final double TOP_HARD_STOP_SLACK_METERS = 0.02;

    public String name() {
      return "Elevator (sim): encoder matches the true height";
    }

    public String[] requiredKeys() {
      return new String[] {
        OUT + "Sim/TrueHeightMeters",
        IO + "PositionMeters",
        IO + "VelocityMetersPerSec",
        OUT + "Homed"
      };
    }

    public void check(LogFile log, Report report) {
      double homedTime = firstTrue(log.booleans(OUT + "Homed"));
      double worstError = 0;
      double topStop = ElevatorConstants.MAX_HEIGHT_METERS + TOP_HARD_STOP_SLACK_METERS;
      for (NumberSample truth : log.numbers(OUT + "Sim/TrueHeightMeters")) {
        if (truth.value > topStop + 1e-6) {
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
    public String name() {
      return "Elevator (sim): demo visits every height";
    }

    public String[] requiredKeys() {
      return new String[] {SimSmokeKeys.AUTO, OUT + "GoalMeters", OUT + "ControlMode"};
    }

    public void check(LogFile log, Report report) {
      List<TextSample> autos = log.texts(SimSmokeKeys.AUTO);
      if (!autos.get(0).value.equals("Elevator Demo")) {
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
      List<Move> moves = findMoves(log);
      StringBuilder visited = new StringBuilder();
      for (Move move : moves) {
        visited.append(String.format(" %.2f", move.goalMeters));
      }
      boolean sameCount = moves.size() == wanted.length;
      for (int i = 0; sameCount && i < wanted.length; i++) {
        if (Math.abs(moves.get(i).goalMeters - wanted[i]) > 1e-6) {
          sameCount = false;
        }
      }
      if (!sameCount) {
        report.error("goals were" + visited + ", expected stow, low, high, mid, stow");
      }
    }
  }

  // ---------------- Shared helpers ----------------

  /** One move: from the time a goal was set until the next goal, or until it stopped holding. */
  static class Move {
    final double startSeconds;
    final double endSeconds;
    final double goalMeters;

    Move(double startSeconds, double endSeconds, double goalMeters) {
      this.startSeconds = startSeconds;
      this.endSeconds = endSeconds;
      this.goalMeters = goalMeters;
    }
  }

  /** Every move in the log: each new goal or entry into POSITION mode starts one. */
  static List<Move> findMoves(LogFile log) {
    List<Double> changeTimes = new ArrayList<>();
    for (TextSample mode : log.texts(OUT + "ControlMode")) {
      changeTimes.add(mode.timeSeconds);
    }
    for (NumberSample goal : log.numbers(OUT + "GoalMeters")) {
      changeTimes.add(goal.timeSeconds);
    }
    changeTimes.add(log.endTimeSeconds());
    changeTimes.sort(null);

    List<Move> moves = new ArrayList<>();
    for (int i = 0; i < changeTimes.size() - 1; i++) {
      double start = changeTimes.get(i);
      double goal = log.numberAt(OUT + "GoalMeters", start);
      boolean holding = modeAt(log, start).equals("POSITION");
      boolean sameAsLast =
          !moves.isEmpty()
              && moves.get(moves.size() - 1).goalMeters == goal
              && moves.get(moves.size() - 1).endSeconds == start;
      if (holding && sameAsLast) {
        Move last = moves.remove(moves.size() - 1);
        moves.add(new Move(last.startSeconds, changeTimes.get(i + 1), goal));
      } else if (holding) {
        moves.add(new Move(start, changeTimes.get(i + 1), goal));
      }
    }
    return moves;
  }

  /** The elevator's control mode at a time, or "" before the first one. */
  static String modeAt(LogFile log, double timeSeconds) {
    String mode = "";
    for (TextSample sample : log.texts(OUT + "ControlMode")) {
      if (sample.timeSeconds > timeSeconds) {
        break;
      }
      mode = sample.value;
    }
    return mode;
  }

  /** When a true/false key first turned true, or NaN if it never did. */
  static double firstTrue(List<BooleanSample> samples) {
    for (BooleanSample sample : samples) {
      if (sample.value) {
        return sample.timeSeconds;
      }
    }
    return Double.NaN;
  }

  /** The largest value in a list, or the largest size (ignoring sign) if useSize is true. */
  static double largest(List<NumberSample> samples, boolean useSize) {
    double largest = 0;
    for (NumberSample sample : samples) {
      double value = useSize ? Math.abs(sample.value) : sample.value;
      largest = Math.max(largest, value);
    }
    return largest;
  }

  /** About how long Motion Magic takes to move a distance, including the jerk ramps. */
  static double profileSeconds(double distanceMeters) {
    double cruise = ElevatorConstants.CRUISE_VELOCITY_MPS;
    double accel = ElevatorConstants.ACCELERATION_MPS2;
    double seconds;
    if (distanceMeters > cruise * cruise / accel) {
      seconds = distanceMeters / cruise + cruise / accel;
    } else {
      seconds = 2 * Math.sqrt(distanceMeters / accel);
    }
    return seconds + 2 * accel / ElevatorConstants.JERK_MPS3;
  }
}
