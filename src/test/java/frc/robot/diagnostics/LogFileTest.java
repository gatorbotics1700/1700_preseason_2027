package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import java.io.IOException;
import java.nio.file.Path;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * LogFile reads back exactly what was written. The test writes a small log with WPILib's own writer
 * (the same one the robot uses), so it needs no robot and no saved log file.
 */
public class LogFileTest {

  private static final double DELTA = 1e-9;

  private static LogFile log;

  /** Writes one small log with a value of each type, then reads it back once for all tests. */
  @BeforeAll
  public static void writeAndReadLog(@TempDir Path folder) throws IOException {
    Path file = folder.resolve("test.wpilog");
    DataLogWriter writer = new DataLogWriter(file.toString());
    DoubleLogEntry height = new DoubleLogEntry(writer, "/Mech/Test/HeightMeters", 0);
    BooleanLogEntry homed = new BooleanLogEntry(writer, "/RealOutputs/Mech/Test/Homed", 0);
    StringLogEntry console = new StringLogEntry(writer, "/RealOutputs/Console", 0);
    StructArrayLogEntry<Pose3d> parts =
        StructArrayLogEntry.create(writer, "/RealOutputs/Mech/Test/ComponentPoses", Pose3d.struct);

    // Times are in microseconds: 1_000_000 is 1 s.
    height.append(0.25, 1_000_000);
    height.append(0.75, 3_000_000);
    homed.append(true, 2_000_000);
    console.append("talon fx 40 (\"\") Status Signal Position", 1_500_000);
    Pose3d stage = new Pose3d(0, 0, 0.5, new Rotation3d());
    Pose3d carriage = new Pose3d(0, 0, 1.0, new Rotation3d());
    parts.append(new Pose3d[] {stage, carriage}, 2_500_000);
    writer.close();

    log = LogFile.read(file);
  }

  /** Numbers come back with their times in seconds. */
  @Test
  public void readsNumbers() {
    assertEquals(2, log.numbers("/Mech/Test/HeightMeters").size());
    assertEquals(1.0, log.numbers("/Mech/Test/HeightMeters").get(0).timeSeconds, DELTA);
    assertEquals(0.75, log.numbers("/Mech/Test/HeightMeters").get(1).value, DELTA);
  }

  /** numberAt gives the last value logged at or before a time, and NaN before the first. */
  @Test
  public void numberAtHoldsTheLastValue() {
    assertTrue(Double.isNaN(log.numberAt("/Mech/Test/HeightMeters", 0.5)));
    assertEquals(0.25, log.numberAt("/Mech/Test/HeightMeters", 2.9), DELTA);
    assertEquals(0.75, log.numberAt("/Mech/Test/HeightMeters", 3.0), DELTA);
  }

  /** True/false values and text come back as written. */
  @Test
  public void readsBooleansAndText() {
    assertFalse(log.booleanAt("/RealOutputs/Mech/Test/Homed", 1.9));
    assertTrue(log.booleanAt("/RealOutputs/Mech/Test/Homed", 2.0));
    assertEquals(
        "talon fx 40 (\"\") Status Signal Position",
        log.texts("/RealOutputs/Console").get(0).value);
  }

  /** Pose lists come back as Pose3d, decoded the way AdvantageScope decodes them. */
  @Test
  public void readsPoses() {
    Pose3d[] parts = log.poses("/RealOutputs/Mech/Test/ComponentPoses").get(0).value;
    assertEquals(2, parts.length);
    assertEquals(0.5, parts[0].getZ(), DELTA);
    assertEquals(1.0, parts[1].getZ(), DELTA);
  }

  /** Missing keys are empty, not errors, so a check can tell "not in this log". */
  @Test
  public void missingKeysAreEmpty() {
    assertFalse(log.has("/Mech/Nothing"));
    assertTrue(log.numbers("/Mech/Nothing").isEmpty());
    assertTrue(log.has("/Mech/Test/HeightMeters"));
  }

  /**
   * The last record is read even when it is short. WPILib's for-each loop over a log drops it
   * (2026.2.1), which once hid a whole key from the checks.
   */
  @Test
  public void readsAShortLastRecord() throws IOException {
    TestLog shortLog = new TestLog();
    shortLog.number("/A", 1.0, 6.0);
    shortLog.number("/B", 1.0, 90.0);

    LogFile read = shortLog.read();

    assertEquals(1, read.numbers("/B").size());
    assertEquals(90.0, read.numbers("/B").get(0).value, DELTA);
  }
}
