package frc.robot.diagnostics;

/**
 * One thing to look for in a robot log, such as "the elevator homes before it moves".
 *
 * <p>An interface is a list of methods a class promises to have. Every check fills in these three,
 * so LogDiagnostics can run any check the same way. To add a check: write a small class that
 * implements this (copy one from ElevatorChecks), then add it to the list in its checks file.
 */
public interface LogCheck {

  /** A short name for the report, starting with the mechanism: "Elevator: homes before moving". */
  String name();

  /**
   * The log keys this check reads. If any is missing from a log, the check is reported as skipped,
   * not failed: a robot without an elevator, or a match log without the simulator's keys.
   */
  String[] requiredKeys();

  /** Looks through the log and reports what it finds with report.error(), warning() or note(). */
  void check(LogFile log, Report report);
}
