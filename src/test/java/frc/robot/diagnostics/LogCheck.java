package frc.robot.diagnostics;

/**
 * One thing to look for in a robot log, such as "the elevator homes before it moves". To add a
 * check, copy one from ElevatorChecks and add it to that file's all() list.
 */
public interface LogCheck {

  /** A short name for the report, starting with the mechanism: "Elevator: homes before moving". */
  String name();

  /** The log keys this check reads; if any is missing, the check is skipped, not failed. */
  String[] requiredKeys();

  /** Looks through the log and reports what it finds with report.error(), warning() or note(). */
  void check(LogFile log, Report report);
}
