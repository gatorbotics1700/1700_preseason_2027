package frc.robot.diagnostics;

import frc.robot.view3d.ElevatorView3dChecks;
import frc.robot.view3d.View3dChecks;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs every check on a robot log (match, practice or simulator) and prints what it found.
 *
 * <p>From a terminal in the project folder:
 *
 * <pre>
 *   ./gradlew diagnoseLog -Plog=path/to/file.wpilog
 *   ./gradlew diagnoseLog -Plog=path/to/folder      (uses the newest .wpilog in it)
 * </pre>
 */
public final class LogDiagnostics {

  private LogDiagnostics() {}

  /** Every check for every mechanism. A new mechanism adds its checks file here. */
  public static List<LogCheck> allChecks() {
    List<LogCheck> checks = new ArrayList<>();
    checks.addAll(RobotChecks.all());
    checks.addAll(View3dChecks.all());
    checks.addAll(ElevatorChecks.all());
    checks.addAll(ElevatorView3dChecks.all());
    return checks;
  }

  /** Runs the checks on a log and returns what they found. */
  public static Report diagnose(LogFile log, List<LogCheck> checks) {
    Report report = new Report();
    for (LogCheck check : checks) {
      report.run(check, log);
    }
    return report;
  }

  /** Checks a log (plus an optional console text file) and exits 1 if any check found an error. */
  public static void main(String[] args) throws IOException {
    if (args.length < 1) {
      System.out.println("Give a log: ./gradlew diagnoseLog -Plog=path/to/file.wpilog");
      System.exit(2);
    }
    LogFile log = LogFile.read(findLog(Path.of(args[0])));
    if (args.length > 1) {
      log.addConsoleText(Files.readString(Path.of(args[1])));
    }
    Report report = diagnose(log, allChecks());
    System.out.println("Log: " + log.getPath() + " (" + log.keys().size() + " keys)");
    System.out.print(report.toText());
    System.exit(report.getErrorCount() > 0 ? 1 : 0);
  }

  /** The log itself, or the newest .wpilog in a folder. */
  public static Path findLog(Path fileOrFolder) throws IOException {
    if (!Files.isDirectory(fileOrFolder)) {
      return fileOrFolder;
    }
    File newest = null;
    for (File file : fileOrFolder.toFile().listFiles()) {
      boolean isLog = file.getName().endsWith(".wpilog");
      if (isLog && (newest == null || file.lastModified() > newest.lastModified())) {
        newest = file;
      }
    }
    if (newest == null) {
      throw new IOException("No .wpilog file in " + fileOrFolder);
    }
    return newest.toPath();
  }
}
