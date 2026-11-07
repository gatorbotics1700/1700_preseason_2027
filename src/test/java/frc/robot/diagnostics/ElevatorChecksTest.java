package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The elevator checks catch what they are meant to catch, and pass a good log. Each test builds a
 * small fake log with TestLog. Expected numbers are typed here, not read from constants.
 */
public class ElevatorChecksTest {

  private static final String IO = "/Mech/Elevator/IO/";
  private static final String OUT = "/RealOutputs/Mech/Elevator/";

  /** Runs one check on a log. */
  private static Report run(LogCheck check, TestLog log) throws IOException {
    return LogDiagnostics.diagnose(log.read(), List.of(check));
  }

  /** A log where the elevator is homed at 1 s and starts holding 0 m. */
  private static TestLog homedAtOneSecond() throws IOException {
    TestLog log = new TestLog();
    log.bool(IO + "BottomLimitPressed", 1.0, true);
    log.bool(OUT + "Homed", 1.0, true);
    log.text(OUT + "ControlMode", 1.0, "POSITION");
    log.number(OUT + "GoalMeters", 1.0, 0.0);
    log.number(IO + "PositionMeters", 1.0, 0.0);
    return log;
  }

  /** A raised goal before homing is an error. */
  @Test
  public void raisedGoalBeforeHomingIsAnError() throws IOException {
    TestLog log = new TestLog();
    log.bool(IO + "BottomLimitPressed", 2.0, true);
    log.number(OUT + "GoalMeters", 1.0, 0.5);
    log.bool(OUT + "Homed", 2.0, true);

    assertEquals(1, run(new ElevatorChecks.HomesBeforeMoving(), log).getErrorCount());
  }

  /** Homing on the switch, then a raised goal, passes. */
  @Test
  public void homingThenMovingPasses() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(OUT + "GoalMeters", 2.0, 0.5);

    assertEquals(0, run(new ElevatorChecks.HomesBeforeMoving(), log).getErrorCount());
  }

  /** A move that settles quickly passes. */
  @Test
  public void quickMovePasses() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(OUT + "GoalMeters", 2.0, 0.4);
    log.number(IO + "PositionMeters", 2.2, 0.2);
    log.number(IO + "PositionMeters", 2.5, 0.4);
    log.number(IO + "PositionMeters", 5.0, 0.4);

    assertEquals(0, run(new ElevatorChecks.MovesReachGoalInTime(), log).getErrorCount());
  }

  /** A move that never gets there, with plenty of time, is an error. */
  @Test
  public void moveThatNeverArrivesIsAnError() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(OUT + "GoalMeters", 2.0, 0.4);
    log.number(IO + "PositionMeters", 2.5, 0.3);
    log.number(IO + "PositionMeters", 6.0, 0.3);

    assertEquals(1, run(new ElevatorChecks.MovesReachGoalInTime(), log).getErrorCount());
  }

  /** A move cut short by a new goal, before it was due, is only a note. */
  @Test
  public void interruptedMoveIsNotAnError() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(OUT + "GoalMeters", 2.0, 1.0);
    log.number(IO + "PositionMeters", 2.2, 0.3);
    log.number(OUT + "GoalMeters", 2.3, 0.3);
    log.number(IO + "PositionMeters", 5.0, 0.3);

    assertEquals(0, run(new ElevatorChecks.MovesReachGoalInTime(), log).getErrorCount());
  }

  /** Going 5 cm past the goal is an error. */
  @Test
  public void overshootIsAnError() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(OUT + "GoalMeters", 2.0, 0.4);
    log.number(IO + "PositionMeters", 2.3, 0.45);
    log.number(IO + "PositionMeters", 2.5, 0.4);
    log.number(IO + "PositionMeters", 5.0, 0.4);

    assertEquals(1, run(new ElevatorChecks.MovesReachGoalInTime(), log).getErrorCount());
  }

  /** A move that settles 3 s late is an error. */
  @Test
  public void slowMoveIsAnError() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(OUT + "GoalMeters", 2.0, 0.4);
    log.number(IO + "PositionMeters", 3.0, 0.2);
    log.number(IO + "PositionMeters", 5.0, 0.4);
    log.number(IO + "PositionMeters", 7.0, 0.4);

    assertEquals(1, run(new ElevatorChecks.MovesReachGoalInTime(), log).getErrorCount());
  }

  /** A height above the top of the travel is an error. */
  @Test
  public void leavingTheTravelIsAnError() throws IOException {
    TestLog log = homedAtOneSecond();
    log.number(IO + "PositionMeters", 2.0, 1.25);

    assertEquals(1, run(new ElevatorChecks.StaysInTravel(), log).getErrorCount());
  }

  /** 90 A against an 80 A limit is an error. */
  @Test
  public void currentOverTheLimitIsAnError() throws IOException {
    TestLog log = new TestLog();
    log.number(IO + "AppliedVolts", 1.0, 6.0);
    log.number(IO + "StatorCurrentAmps", 1.0, 90.0);

    assertEquals(1, run(new ElevatorChecks.StaysInVoltageAndCurrentLimits(), log).getErrorCount());
  }

  /** A log without the elevator skips its checks instead of failing them. */
  @Test
  public void logWithoutElevatorSkips() throws IOException {
    TestLog log = new TestLog();
    log.number("/Drive/Module0/DriveVelocity", 1.0, 2.0);

    Report report = LogDiagnostics.diagnose(log.read(), ElevatorChecks.all());

    assertEquals(0, report.getErrorCount());
    assertTrue(report.toText().startsWith("SKIP"));
  }
}
