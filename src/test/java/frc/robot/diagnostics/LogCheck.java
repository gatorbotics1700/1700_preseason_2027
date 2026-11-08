package frc.robot.diagnostics;

/**
 * One thing to look for in a robot log, such as "every device stays connected". To add a check,
 * copy one from RobotChecks and add it to its file's all() list.
 */
public interface LogCheck {

  /** A short name for the report, starting with the mechanism: "Robot: devices stay connected". */
  String name();

  /** The log keys this check reads; if any is missing, the check is skipped, not failed. */
  String[] requiredKeys();

  /** Looks through the log and reports what it finds with report.error(), warning() or note(). */
  void check(LogFile log, Report report);
}
