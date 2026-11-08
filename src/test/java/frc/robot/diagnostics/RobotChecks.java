package frc.robot.diagnostics;

import frc.robot.diagnostics.LogFile.BooleanSample;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Things to look for in any robot log, whatever mechanisms it has. */
public final class RobotChecks {

  private RobotChecks() {}

  /** Every whole-robot check. */
  public static List<LogCheck> all() {
    List<LogCheck> checks = new ArrayList<>();
    checks.add(new DevicesStayConnected());
    checks.add(new CanDevicesRespond());
    return checks;
  }

  /** Every "...Connected" value stays true; never connecting is a warning (some sim devices). */
  static class DevicesStayConnected implements LogCheck {
    /** The check's name in the report. */
    public String name() {
      return "Robot: devices stay connected";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      for (String key : log.keys()) {
        if (key.endsWith("Connected") && !log.booleans(key).isEmpty()) {
          checkDevice(key, log.booleans(key), report);
        }
      }
    }

    /** Reports a device that dropped out after connecting, or never connected. */
    private static void checkDevice(String key, List<BooleanSample> samples, Report report) {
      boolean wasConnected = false;
      for (BooleanSample sample : samples) {
        if (sample.value) {
          wasConnected = true;
        } else if (wasConnected) {
          report.error(String.format("%s dropped out at %.2f s", key, sample.timeSeconds));
          return;
        }
      }
      if (!wasConnected) {
        report.warning(key + " never connected");
      }
    }
  }

  /** No CTRE device repeats a console error; one each at boot is normal, a repeat is not. */
  static class CanDevicesRespond implements LogCheck {
    // Phoenix prints errors like: talon fx 40 ("") Status Signal Position
    private static final Pattern DEVICE_ERROR =
        Pattern.compile("(talon fx|cancoder|pigeon 2) (\\d+) \\(\"[^\"]*\"\\) (.+)");

    /** The check's name in the report. */
    public String name() {
      return "Robot: CAN devices respond";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      Map<String, Integer> counts = new HashMap<>();
      Matcher matcher = DEVICE_ERROR.matcher(log.consoleText());
      while (matcher.find()) {
        String error = matcher.group(1) + " " + matcher.group(2) + ": " + matcher.group(3).trim();
        counts.put(error, counts.getOrDefault(error, 0) + 1);
      }
      int total = 0;
      for (String error : counts.keySet()) {
        total += counts.get(error);
        if (counts.get(error) > 1) {
          report.error(error + " (" + counts.get(error) + " times)");
        }
      }
      report.note(total + " CAN errors in the console");
    }
  }
}
