package frc.robot.diagnostics;

import frc.robot.diagnostics.LogFile.BooleanSample;
import frc.robot.diagnostics.LogFile.NumberSample;
import frc.robot.diagnostics.LogFile.TextSample;
import java.util.ArrayList;
import java.util.List;

/**
 * The standard log checks for a mechanism that moves to positions (elevator, arm, hood, turret,
 * climber). Fill in a Settings with the mechanism's log keys and limits, then add all(settings) to
 * LogDiagnostics.
 */
public final class PositionChecks {

  /** What the checks need to know about one mechanism: its log keys and its limits. */
  public static class Settings {
    /** Starts the report's check names, for example "Elevator". */
    public String name;

    /** The position unit for the report, for example "m" or "deg". */
    public String unit;

    // Log keys.
    public String positionKey;
    public String goalKey;
    public String modeKey;
    public String homedKey;
    public String homeSwitchKey;
    public String voltsKey;
    public String currentKey;

    /** The mode name the mechanism logs while it holds a position. */
    public String holdingMode = "POSITION";

    // Limits, in the position unit.
    public double homePosition;
    public double minPosition;
    public double maxPosition;
    public double tolerance;
    public double maxOvershoot;

    /** How far past the travel limits still counts as inside, for sensor noise. */
    public double travelSlack;

    // The motion profile, in the position unit per second (and per second squared, cubed).
    public double cruiseVelocity;
    public double acceleration;
    public double jerk;

    public double statorCurrentLimitAmps;
  }

  /** One move: from the time a goal was set until the next goal, or until it stopped holding. */
  public static class Move {
    public final double startSeconds;
    public final double endSeconds;
    public final double goal;

    /** A move toward a goal, from start to end. */
    Move(double startSeconds, double endSeconds, double goal) {
      this.startSeconds = startSeconds;
      this.endSeconds = endSeconds;
      this.goal = goal;
    }
  }

  private PositionChecks() {}

  /** The four standard checks for one mechanism. */
  public static List<LogCheck> all(Settings settings) {
    List<LogCheck> checks = new ArrayList<>();
    checks.add(new HomesBeforeMoving(settings));
    checks.add(new MovesReachGoalInTime(settings));
    checks.add(new StaysInTravel(settings));
    checks.add(new StaysInVoltageAndCurrentLimits(settings));
    return checks;
  }

  /** The home switch closes and Homed turns on before any goal away from home is set. */
  static class HomesBeforeMoving implements LogCheck {
    private final Settings settings;

    /** Checks this mechanism. */
    HomesBeforeMoving(Settings settings) {
      this.settings = settings;
    }

    /** The check's name in the report. */
    public String name() {
      return settings.name + ": homes before moving";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {settings.homeSwitchKey, settings.homedKey, settings.goalKey};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      double homedTime = firstTrue(log.booleans(settings.homedKey));
      double switchTime = firstTrue(log.booleans(settings.homeSwitchKey));
      for (NumberSample goal : log.numbers(settings.goalKey)) {
        boolean awayFromHome = Math.abs(goal.value - settings.homePosition) > settings.tolerance;
        if (awayFromHome && !(goal.timeSeconds >= homedTime)) {
          report.error(
              String.format(
                  "goal %.2f %s at %.2f s, before homing",
                  goal.value, settings.unit, goal.timeSeconds));
        }
      }
      if (Double.isNaN(homedTime)) {
        report.note("never homed");
      } else if (Double.isNaN(switchTime) || switchTime > homedTime) {
        report.error(String.format("homed at %.2f s without the home switch closing", homedTime));
      } else {
        report.note(String.format("homed at %.2f s", homedTime));
      }
    }
  }

  /** Each move settles within tolerance in about its profile time, without overshoot. */
  static class MovesReachGoalInTime implements LogCheck {
    // Extra time allowed on top of the profile, for the last bit of settling.
    private static final double SETTLE_MARGIN_SECONDS = 0.6;

    private final Settings settings;

    /** Checks this mechanism. */
    MovesReachGoalInTime(Settings settings) {
      this.settings = settings;
    }

    /** The check's name in the report. */
    public String name() {
      return settings.name + ": moves reach the goal in time";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {settings.positionKey, settings.goalKey, settings.modeKey};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      for (Move move : findMoves(log, settings)) {
        checkMove(log, move, report);
      }
    }

    /** Checks one move: settled, how long it took, and how far past the goal it went. */
    private void checkMove(LogFile log, Move move, Report report) {
      double start = log.numberAt(settings.positionKey, move.startSeconds);
      double allowedSeconds =
          profileSeconds(settings, Math.abs(move.goal - start)) + SETTLE_MARGIN_SECONDS;
      double settledSeconds = settledTime(log, move);
      String what =
          String.format(
              "move %.2f -> %.2f %s at %.2f s", start, move.goal, settings.unit, move.startSeconds);

      if (Double.isNaN(settledSeconds)) {
        reportNotSettled(move, allowedSeconds, what, report);
        return;
      }
      reportSettleTime(settledSeconds - move.startSeconds, allowedSeconds, what, report);
      double overshoot = overshoot(log, move, start);
      if (overshoot > settings.maxOvershoot) {
        report.error(
            String.format("%s: went %.3f %s past the goal", what, overshoot, settings.unit));
      }
    }

    /** A move that never settled: an error if it had time to, a note if a new goal cut it off. */
    private static void reportNotSettled(
        Move move, double allowedSeconds, String what, Report report) {
      if (move.endSeconds - move.startSeconds >= allowedSeconds) {
        report.error(what + ": never reached the goal");
      } else {
        report.note(what + ": interrupted before it was due");
      }
    }

    /** How long a move took to settle: an error if longer than allowed, a note otherwise. */
    private static void reportSettleTime(
        double tookSeconds, double allowedSeconds, String what, Report report) {
      String timing =
          String.format(
              "%s: settled in %.2f s (allowed %.2f s)", what, tookSeconds, allowedSeconds);
      if (tookSeconds > allowedSeconds) {
        report.error(timing);
      } else {
        report.note(timing);
      }
    }

    /** The time after which the position stays within tolerance until the move ends, or NaN. */
    private double settledTime(LogFile log, Move move) {
      // Positions are logged only when they change, so start from the position at the start.
      double start = log.numberAt(settings.positionKey, move.startSeconds);
      double settled = Double.NaN;
      if (Math.abs(start - move.goal) <= settings.tolerance) {
        settled = move.startSeconds;
      }
      for (NumberSample sample : log.numbers(settings.positionKey)) {
        if (sample.timeSeconds < move.startSeconds || sample.timeSeconds > move.endSeconds) {
          continue;
        }
        boolean inTolerance = Math.abs(sample.value - move.goal) <= settings.tolerance;
        if (!inTolerance) {
          settled = Double.NaN;
        } else if (Double.isNaN(settled)) {
          settled = sample.timeSeconds;
        }
      }
      return settled;
    }

    /** How far past the goal the position went during the move, in its direction of travel. */
    private double overshoot(LogFile log, Move move, double start) {
      boolean goingUp = move.goal >= start;
      double worst = 0;
      for (NumberSample sample : log.numbers(settings.positionKey)) {
        if (sample.timeSeconds >= move.startSeconds && sample.timeSeconds <= move.endSeconds) {
          double past = move.goal - sample.value;
          if (goingUp) {
            past = sample.value - move.goal;
          }
          worst = Math.max(worst, past);
        }
      }
      return worst;
    }
  }

  /** After homing, the position stays inside the travel limits. */
  static class StaysInTravel implements LogCheck {
    // The position reset lands one loop after the switch closes, so skip a little after homing.
    private static final double AFTER_HOMING_SECONDS = 0.03;

    private final Settings settings;

    /** Checks this mechanism. */
    StaysInTravel(Settings settings) {
      this.settings = settings;
    }

    /** The check's name in the report. */
    public String name() {
      return settings.name + ": stays inside its travel";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {settings.positionKey, settings.homedKey};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      double homedTime = firstTrue(log.booleans(settings.homedKey));
      double lowest = settings.minPosition - settings.travelSlack;
      double highest = settings.maxPosition + settings.travelSlack;
      for (NumberSample sample : log.numbers(settings.positionKey)) {
        boolean afterHoming = sample.timeSeconds > homedTime + AFTER_HOMING_SECONDS;
        if (afterHoming && (sample.value < lowest || sample.value > highest)) {
          report.error(
              String.format(
                  "position %.3f %s at %.2f s", sample.value, settings.unit, sample.timeSeconds));
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

    private final Settings settings;

    /** Checks this mechanism. */
    StaysInVoltageAndCurrentLimits(Settings settings) {
      this.settings = settings;
    }

    /** The check's name in the report. */
    public String name() {
      return settings.name + ": stays inside voltage and current limits";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {settings.voltsKey, settings.currentKey};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      double maxVolts = largest(log.numbers(settings.voltsKey), true);
      double maxAmps = largest(log.numbers(settings.currentKey), false);
      double ampLimit = settings.statorCurrentLimitAmps + CURRENT_SLACK_AMPS;
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

  // ---------------- Shared helpers ----------------

  /** Every move in the log: each new goal, or entry into the holding mode, starts one. */
  public static List<Move> findMoves(LogFile log, Settings settings) {
    List<Double> changeTimes = changeTimes(log, settings);
    List<Move> moves = new ArrayList<>();
    for (int i = 0; i < changeTimes.size() - 1; i++) {
      double start = changeTimes.get(i);
      double goal = log.numberAt(settings.goalKey, start);
      boolean holding = modeAt(log, settings, start).equals(settings.holdingMode);
      boolean sameAsLast =
          !moves.isEmpty()
              && moves.get(moves.size() - 1).goal == goal
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

  /** Every time the mode or the goal changed, plus the end of the log, in order. */
  static List<Double> changeTimes(LogFile log, Settings settings) {
    List<Double> times = new ArrayList<>();
    for (TextSample mode : log.texts(settings.modeKey)) {
      times.add(mode.timeSeconds);
    }
    for (NumberSample goal : log.numbers(settings.goalKey)) {
      times.add(goal.timeSeconds);
    }
    times.add(log.endTimeSeconds());
    times.sort(null);
    return times;
  }

  /** The mechanism's mode at a time, or "" before the first one. */
  static String modeAt(LogFile log, Settings settings, double timeSeconds) {
    String mode = "";
    for (TextSample sample : log.texts(settings.modeKey)) {
      if (sample.timeSeconds > timeSeconds) {
        break;
      }
      mode = sample.value;
    }
    return mode;
  }

  /** When a true/false key first turned true, or NaN if it never did. */
  public static double firstTrue(List<BooleanSample> samples) {
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
      double value = sample.value;
      if (useSize) {
        value = Math.abs(sample.value);
      }
      largest = Math.max(largest, value);
    }
    return largest;
  }

  /** About how long the motion profile takes to move a distance, including the jerk ramps. */
  static double profileSeconds(Settings settings, double distance) {
    double cruise = settings.cruiseVelocity;
    double accel = settings.acceleration;
    double seconds;
    if (distance > cruise * cruise / accel) {
      seconds = distance / cruise + cruise / accel;
    } else {
      seconds = 2 * Math.sqrt(distance / accel);
    }
    return seconds + 2 * accel / settings.jerk;
  }
}
