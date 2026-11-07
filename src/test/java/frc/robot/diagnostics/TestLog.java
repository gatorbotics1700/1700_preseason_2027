package frc.robot.diagnostics;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

/**
 * Builds a small fake log for testing a check: add the values the check reads, then read() it.
 *
 * <pre>
 *   TestLog log = new TestLog();
 *   log.number("/Mech/Elevator/IO/PositionMeters", 1.0, 0.5);   // key, time in seconds, value
 *   Report report = LogDiagnostics.diagnose(log.read(), List.of(new MyCheck()));
 * </pre>
 *
 * <p>Values for one key must be added oldest first, like a real log.
 */
public class TestLog {

  private static final double MICROSECONDS_PER_SECOND = 1e6;

  private final Path file;
  private final DataLogWriter writer;
  private final Map<String, DoubleLogEntry> numbers = new HashMap<>();
  private final Map<String, BooleanLogEntry> booleans = new HashMap<>();
  private final Map<String, StringLogEntry> texts = new HashMap<>();

  /** Starts an empty log in a temporary file. */
  public TestLog() throws IOException {
    file = Files.createTempFile("test", ".wpilog");
    writer = new DataLogWriter(file.toString());
  }

  /** Adds a number at a time. */
  public void number(String key, double timeSeconds, double value) {
    if (!numbers.containsKey(key)) {
      numbers.put(key, new DoubleLogEntry(writer, key, 0));
    }
    numbers.get(key).append(value, toMicroseconds(timeSeconds));
  }

  /** Adds a true/false at a time. */
  public void bool(String key, double timeSeconds, boolean value) {
    if (!booleans.containsKey(key)) {
      booleans.put(key, new BooleanLogEntry(writer, key, 0));
    }
    booleans.get(key).append(value, toMicroseconds(timeSeconds));
  }

  /** Adds text at a time. */
  public void text(String key, double timeSeconds, String value) {
    if (!texts.containsKey(key)) {
      texts.put(key, new StringLogEntry(writer, key, 0));
    }
    texts.get(key).append(value, toMicroseconds(timeSeconds));
  }

  /** Finishes the file and reads it back, the way the checker reads a real log. */
  public LogFile read() throws IOException {
    writer.close();
    LogFile log = LogFile.read(file);
    Files.delete(file);
    return log;
  }

  private static long toMicroseconds(double timeSeconds) {
    return Math.round(timeSeconds * MICROSECONDS_PER_SECOND);
  }
}
