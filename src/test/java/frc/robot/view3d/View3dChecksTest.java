package frc.robot.view3d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.diagnostics.LogDiagnostics;
import frc.robot.diagnostics.Report;
import frc.robot.diagnostics.SimSmokeKeys;
import frc.robot.diagnostics.TestLog;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The layout check catches layout keys the robot never logs. */
public class View3dChecksTest {

  /** A layout with one 3D tab that reads a robot pose and one mechanism's poses. */
  private static Path writeLayout(Path folder) throws IOException {
    String json =
        "{\"hubs\": [{\"state\": {\"tabs\": {\"tabs\": [{\"type\": 3, \"controller\": {"
            + "\"sources\": [{\"type\": \"robot\", \"logKey\": \"/RealOutputs/Robot\"},"
            + " {\"type\": \"component\", \"logKey\": \"/RealOutputs/Mech/Arm/ComponentPoses\"}]"
            + "}}]}}}]}";
    Path layout = folder.resolve("layout.json");
    Files.writeString(layout, json);
    return layout;
  }

  /** Every layout key in the log passes. */
  @Test
  public void layoutKeysInTheLogPass(@TempDir Path folder) throws IOException {
    TestLog log = new TestLog();
    log.text(SimSmokeKeys.AUTO, 1.0, "Some Auto");
    log.number("/RealOutputs/Robot", 1.0, 0.0);
    log.number("/RealOutputs/Mech/Arm/ComponentPoses", 1.0, 0.0);

    View3dChecks.LayoutKeysAreLogged check =
        new View3dChecks.LayoutKeysAreLogged(writeLayout(folder));
    Report report = LogDiagnostics.diagnose(log.read(), List.of(check));

    assertEquals(0, report.getErrorCount());
  }

  /** A layout key missing from the log is an error. */
  @Test
  public void layoutKeyMissingFromTheLogIsAnError(@TempDir Path folder) throws IOException {
    TestLog log = new TestLog();
    log.text(SimSmokeKeys.AUTO, 1.0, "Some Auto");
    log.number("/RealOutputs/Robot", 1.0, 0.0);

    View3dChecks.LayoutKeysAreLogged check =
        new View3dChecks.LayoutKeysAreLogged(writeLayout(folder));
    Report report = LogDiagnostics.diagnose(log.read(), List.of(check));

    assertEquals(1, report.getErrorCount());
  }
}
