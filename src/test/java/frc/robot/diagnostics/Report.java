package frc.robot.diagnostics;

import java.util.ArrayList;
import java.util.List;

/** What the checks found in one log: errors (sim smoke fails on them), warnings and notes. */
public class Report {

  /** How serious a finding is. */
  public enum Level {
    ERROR,
    WARNING,
    NOTE
  }

  /** One thing a check found. */
  public static class Finding {
    public final String checkName;
    public final Level level;
    public final String message;

    Finding(String checkName, Level level, String message) {
      this.checkName = checkName;
      this.level = level;
      this.message = message;
    }
  }

  private final List<Finding> findings = new ArrayList<>();
  private final List<String> lines = new ArrayList<>();
  private String currentCheck = "";
  private int errorCount = 0;
  private int warningCount = 0;

  /** Says something is wrong. */
  public void error(String message) {
    findings.add(new Finding(currentCheck, Level.ERROR, message));
    errorCount++;
  }

  /** Says something is worth a look, but not wrong. */
  public void warning(String message) {
    findings.add(new Finding(currentCheck, Level.WARNING, message));
    warningCount++;
  }

  /** Writes down a useful number or fact. */
  public void note(String message) {
    findings.add(new Finding(currentCheck, Level.NOTE, message));
  }

  /** Runs one check and adds its result line: PASS, FAIL, WARN or SKIP. */
  void run(LogCheck check, LogFile log) {
    String missingKey = firstMissingKey(check, log);
    if (missingKey != null) {
      lines.add("SKIP  " + check.name() + "  (not in this log: " + missingKey + ")");
      return;
    }
    currentCheck = check.name();
    int errorsBefore = errorCount;
    int warningsBefore = warningCount;
    int firstFinding = findings.size();
    check.check(log, this);

    String status = "PASS";
    if (errorCount > errorsBefore) {
      status = "FAIL";
    } else if (warningCount > warningsBefore) {
      status = "WARN";
    }
    lines.add(status + "  " + check.name());
    for (int i = firstFinding; i < findings.size(); i++) {
      Finding finding = findings.get(i);
      lines.add("        " + finding.level + ": " + finding.message);
    }
  }

  /** The first key the check needs that the log does not have, or null if it has them all. */
  private static String firstMissingKey(LogCheck check, LogFile log) {
    for (String key : check.requiredKeys()) {
      if (!log.has(key)) {
        return key;
      }
    }
    return null;
  }

  /** Every finding so far. */
  public List<Finding> getFindings() {
    return findings;
  }

  /** How many errors were found. */
  public int getErrorCount() {
    return errorCount;
  }

  /** How many warnings were found. */
  public int getWarningCount() {
    return warningCount;
  }

  /** The whole report as text: one line per check, its findings, then a total. */
  public String toText() {
    StringBuilder text = new StringBuilder();
    for (String line : lines) {
      text.append(line).append('\n');
    }
    text.append(errorCount).append(" errors, ").append(warningCount).append(" warnings\n");
    return text.toString();
  }
}
