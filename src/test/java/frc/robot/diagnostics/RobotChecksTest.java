package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

/** The whole-robot checks catch what they are meant to catch, and pass a good log. */
public class RobotChecksTest {

  /** Runs one check on a log. */
  private static Report run(LogCheck check, LogFile log) {
    return LogDiagnostics.diagnose(log, List.of(check));
  }

  /** A device that drops out after connecting is an error. */
  @Test
  public void deviceDroppingOutIsAnError() throws IOException {
    TestLog log = new TestLog();
    log.bool("/Mech/Elevator/IO/MotorConnected", 1.0, true);
    log.bool("/Mech/Elevator/IO/MotorConnected", 5.0, false);

    assertEquals(1, run(new RobotChecks.DevicesStayConnected(), log.read()).getErrorCount());
  }

  /** A device that never connects is a warning, not an error. */
  @Test
  public void deviceNeverConnectingIsAWarning() throws IOException {
    TestLog log = new TestLog();
    log.bool("/Drive/Gyro/Connected", 1.0, false);

    Report report = run(new RobotChecks.DevicesStayConnected(), log.read());

    assertEquals(0, report.getErrorCount());
    assertEquals(1, report.getWarningCount());
  }

  /** The same CAN error twice is an error; one error per device is normal at boot. */
  @Test
  public void repeatedCanErrorIsAnError() throws IOException {
    LogFile log = new TestLog().read();
    log.addConsoleText("talon fx 17 (\"\") Status Signal Position");
    log.addConsoleText("talon fx 40 (\"\") Status Signal Velocity");
    log.addConsoleText("talon fx 40 (\"\") Status Signal Velocity");

    assertEquals(1, run(new RobotChecks.CanDevicesRespond(), log).getErrorCount());
  }

  /** The console saved inside the log (from the real robot) is checked too. */
  @Test
  public void canErrorsInTheLogConsoleAreFound() throws IOException {
    TestLog log = new TestLog();
    log.text(LogFile.CONSOLE_KEY, 1.0, "talon fx 100 (\"\") Status Signal Position");
    log.text(LogFile.CONSOLE_KEY, 2.0, "talon fx 100 (\"\") Status Signal Position");

    assertEquals(1, run(new RobotChecks.CanDevicesRespond(), log.read()).getErrorCount());
  }
}
