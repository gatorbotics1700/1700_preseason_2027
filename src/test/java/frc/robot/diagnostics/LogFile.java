package frc.robot.diagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;
import edu.wpi.first.util.struct.StructBuffer;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

/**
 * A robot log file (.wpilog), read into memory so checks can look at any logged value over time.
 * AdvantageKit logs a value only when it changes, so use numberAt() to ask what it was at a time.
 */
public class LogFile {

  /** One logged number, and when it was logged. */
  public static class NumberSample {
    public final double timeSeconds;
    public final double value;

    NumberSample(double timeSeconds, double value) {
      this.timeSeconds = timeSeconds;
      this.value = value;
    }
  }

  /** One logged true/false, and when it was logged. */
  public static class BooleanSample {
    public final double timeSeconds;
    public final boolean value;

    BooleanSample(double timeSeconds, boolean value) {
      this.timeSeconds = timeSeconds;
      this.value = value;
    }
  }

  /** One logged piece of text, and when it was logged. */
  public static class TextSample {
    public final double timeSeconds;
    public final String value;

    TextSample(double timeSeconds, String value) {
      this.timeSeconds = timeSeconds;
      this.value = value;
    }
  }

  /** One logged list of 3D poses (for example a mechanism's ComponentPoses), and when. */
  public static class PosesSample {
    public final double timeSeconds;
    public final Pose3d[] value;

    PosesSample(double timeSeconds, Pose3d[] value) {
      this.timeSeconds = timeSeconds;
      this.value = value;
    }
  }

  /** One logged 2D pose (for example the robot on the field), and when it was logged. */
  public static class Pose2dSample {
    public final double timeSeconds;
    public final Pose2d value;

    Pose2dSample(double timeSeconds, Pose2d value) {
      this.timeSeconds = timeSeconds;
      this.value = value;
    }
  }

  private static final double MICROSECONDS_PER_SECOND = 1e6;
  // AdvantageKit saves the robot's console output under this key.
  public static final String CONSOLE_KEY = "/RealOutputs/Console";

  private final Path path;
  // The type of each key, as the log names it: "double", "boolean", "struct:Pose3d[]"...
  private final Map<String, String> types = new HashMap<>();
  // The log names each key once (a "start" record), then refers to it by a number.
  private final Map<Integer, StartRecordData> keysByNumber = new HashMap<>();
  private final Map<String, List<NumberSample>> numbers = new HashMap<>();
  private final Map<String, List<BooleanSample>> booleans = new HashMap<>();
  private final Map<String, List<TextSample>> texts = new HashMap<>();
  private final Map<String, List<PosesSample>> poses = new HashMap<>();
  private final Map<String, List<Pose2dSample>> pose2ds = new HashMap<>();
  // Console text from outside the log, added with addConsoleText().
  private final StringBuilder extraConsoleText = new StringBuilder();

  private LogFile(Path path) {
    this.path = path;
  }

  /** Reads a whole .wpilog file. Fails if it is not a valid log. */
  public static LogFile read(Path path) throws IOException {
    DataLogReader reader = new DataLogReader(path.toString());
    if (!reader.isValid()) {
      throw new IOException("Not a valid .wpilog file: " + path);
    }
    LogFile log = new LogFile(path);
    // forEachRemaining, not for-each: WPILib's iterator skips a short last record.
    reader.iterator().forEachRemaining(record -> log.addRecord(record));
    return log;
  }

  /** Learns a key's name from its start record, or stores a value record under its key. */
  private void addRecord(DataLogRecord record) {
    if (record.isStart()) {
      StartRecordData start = record.getStartData();
      keysByNumber.put(start.entry, start);
      types.put(start.name, start.type);
    } else if (!record.isControl() && keysByNumber.containsKey(record.getEntry())) {
      add(keysByNumber.get(record.getEntry()), record);
    }
  }

  /** Stores one record under its key, decoded by the key's type. Other types are skipped. */
  private void add(StartRecordData key, DataLogRecord record) {
    double timeSeconds = record.getTimestamp() / MICROSECONDS_PER_SECOND;
    String type = key.type;
    if (type.equals("double") || type.equals("float") || type.equals("int64")) {
      double value = readNumber(type, record);
      numbers
          .computeIfAbsent(key.name, name -> new ArrayList<>())
          .add(new NumberSample(timeSeconds, value));
    } else if (type.equals("boolean")) {
      booleans
          .computeIfAbsent(key.name, name -> new ArrayList<>())
          .add(new BooleanSample(timeSeconds, record.getBoolean()));
    } else if (type.equals("string")) {
      texts
          .computeIfAbsent(key.name, name -> new ArrayList<>())
          .add(new TextSample(timeSeconds, record.getString()));
    } else if (type.equals("struct:Pose2d")) {
      Pose2d value = StructBuffer.create(Pose2d.struct).read(ByteBuffer.wrap(record.getRaw()));
      pose2ds
          .computeIfAbsent(key.name, name -> new ArrayList<>())
          .add(new Pose2dSample(timeSeconds, value));
    } else if (type.equals("struct:Pose3d[]") || type.equals("struct:Pose3d")) {
      Pose3d[] value = readPoses(record);
      poses
          .computeIfAbsent(key.name, name -> new ArrayList<>())
          .add(new PosesSample(timeSeconds, value));
    }
  }

  /** Reads a number record of any number type as a double. */
  private static double readNumber(String type, DataLogRecord record) {
    if (type.equals("float")) {
      return record.getFloat();
    } else if (type.equals("int64")) {
      return record.getInteger();
    }
    return record.getDouble();
  }

  /** Reads one Pose3d or a list of them, with the same code the robot uses to write them. */
  private static Pose3d[] readPoses(DataLogRecord record) {
    StructBuffer<Pose3d> buffer = StructBuffer.create(Pose3d.struct);
    ByteBuffer bytes = ByteBuffer.wrap(record.getRaw());
    return buffer.readArray(bytes);
  }

  // ---------------- Reading values ----------------

  /** Where this log was read from. */
  public Path getPath() {
    return path;
  }

  /** Every key in the log, sorted. */
  public Set<String> keys() {
    return new TreeSet<>(types.keySet());
  }

  /** True if the log has this key with at least one value. */
  public boolean has(String key) {
    return numbers.containsKey(key)
        || booleans.containsKey(key)
        || texts.containsKey(key)
        || poses.containsKey(key)
        || pose2ds.containsKey(key);
  }

  /** Every value of a number key, oldest first. Empty if the key is missing. */
  public List<NumberSample> numbers(String key) {
    return numbers.getOrDefault(key, List.of());
  }

  /** Every value of a true/false key, oldest first. Empty if the key is missing. */
  public List<BooleanSample> booleans(String key) {
    return booleans.getOrDefault(key, List.of());
  }

  /** Every value of a text key, oldest first. Empty if the key is missing. */
  public List<TextSample> texts(String key) {
    return texts.getOrDefault(key, List.of());
  }

  /** Every value of a pose list key, oldest first. Empty if the key is missing. */
  public List<PosesSample> poses(String key) {
    return poses.getOrDefault(key, List.of());
  }

  /** Every value of a 2D pose key, oldest first. Empty if the key is missing. */
  public List<Pose2dSample> pose2ds(String key) {
    return pose2ds.getOrDefault(key, List.of());
  }

  /** The value of a number key at a time: the last one logged at or before it. NaN if none. */
  public double numberAt(String key, double timeSeconds) {
    double value = Double.NaN;
    for (NumberSample sample : numbers(key)) {
      if (sample.timeSeconds > timeSeconds) {
        break;
      }
      value = sample.value;
    }
    return value;
  }

  /** The value of a true/false key at a time: the last one logged at or before it. */
  public boolean booleanAt(String key, double timeSeconds) {
    boolean value = false;
    for (BooleanSample sample : booleans(key)) {
      if (sample.timeSeconds > timeSeconds) {
        break;
      }
      value = sample.value;
    }
    return value;
  }

  /** Adds console text from outside the log, such as the simulator's motor controller errors. */
  public void addConsoleText(String text) {
    extraConsoleText.append(text).append('\n');
  }

  /** Everything the robot printed: the log's console key plus any text added from outside. */
  public String consoleText() {
    StringBuilder text = new StringBuilder();
    for (TextSample sample : texts(CONSOLE_KEY)) {
      text.append(sample.value).append('\n');
    }
    text.append(extraConsoleText);
    return text.toString();
  }

  /** The time of the last value of any key: roughly how long the log runs. */
  public double endTimeSeconds() {
    double end = 0;
    for (List<NumberSample> list : numbers.values()) {
      end = Math.max(end, list.get(list.size() - 1).timeSeconds);
    }
    return end;
  }
}
