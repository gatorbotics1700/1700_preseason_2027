package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The elevator's checks read the elevator's keys, and its own checks catch what they should. The
 * standard position checks are tested in PositionChecksTest.
 */
public class ElevatorChecksTest {

  private static final String IO = "/Mech/Elevator/IO/";
  private static final String OUT = "/RealOutputs/Mech/Elevator/";

  /** A goal before homing, logged with the elevator's keys, is caught by its standard checks. */
  @Test
  public void standardChecksReadTheElevatorsKeys() throws IOException {
    TestLog log = new TestLog();
    log.bool(IO + "BottomLimitPressed", 2.0, true);
    log.number(OUT + "GoalMeters", 1.0, 0.5);
    log.bool(OUT + "Homed", 2.0, true);

    Report report = LogDiagnostics.diagnose(log.read(), ElevatorChecks.all());

    assertTrue(report.toText().contains("FAIL  Elevator: homes before moving"));
  }

  /** A log without the elevator skips its checks instead of failing them. */
  @Test
  public void logWithoutElevatorSkips() throws IOException {
    TestLog log = new TestLog();
    log.number("/Drive/Module0/DriveVelocity", 1.0, 2.0);

    Report report = LogDiagnostics.diagnose(log.read(), ElevatorChecks.all());

    assertEquals(0, report.getErrorCount());
  }

  /** In the simulator, an encoder 1 cm off the true height while still is an error. */
  @Test
  public void encoderOffTheTrueHeightIsAnError() throws IOException {
    TestLog log = new TestLog();
    log.bool(OUT + "Homed", 1.0, true);
    log.number(IO + "VelocityMetersPerSec", 1.0, 0.0);
    log.number(IO + "PositionMeters", 1.0, 0.50);
    log.number(OUT + "Sim/TrueHeightMeters", 2.0, 0.51);

    Report report =
        LogDiagnostics.diagnose(log.read(), List.of(new ElevatorChecks.SimHeightMatchesEncoder()));

    assertEquals(1, report.getErrorCount());
  }
}
