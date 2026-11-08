package frc.robot.view3d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.diagnostics.LogDiagnostics;
import frc.robot.diagnostics.LogFile;
import frc.robot.diagnostics.Report;
import frc.robot.diagnostics.SimSmokeKeys;
import frc.robot.diagnostics.TestLog;
import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

/** The 3D view checks catch a model that would not match the code, and pass one that does. */
public class View3dChecksTest {

  private static final String POSITION = "/Mech/Elevator/IO/PositionMeters";
  private static final String POSES = "/RealOutputs/Mech/Elevator/ComponentPoses";

  /** Builds the poses for stage 1 and the carriage by hand. */
  private static Pose3d[] poses(double stage1Meters, double carriageMeters) {
    Pose3d stage1 = new Pose3d(0, 0, stage1Meters, new Rotation3d());
    Pose3d carriage = new Pose3d(0, 0, carriageMeters, new Rotation3d());
    return new Pose3d[] {stage1, carriage};
  }

  /** Poses that follow the height pass. */
  @Test
  public void posesThatFollowTheHeightPass() throws IOException {
    TestLog log = new TestLog();
    log.number(POSITION, 1.0, 0.8);
    log.poses(POSES, 1.0, poses(0.4, 0.8));

    Report report =
        LogDiagnostics.diagnose(log.read(), List.of(new View3dChecks.ElevatorPosesMatchHeight()));

    assertEquals(0, report.getErrorCount());
  }

  /** Stage 1 drawn at the carriage's height (no cascade) is an error. */
  @Test
  public void posesThatIgnoreTheCascadeAreAnError() throws IOException {
    TestLog log = new TestLog();
    log.number(POSITION, 1.0, 0.8);
    log.poses(POSES, 1.0, poses(0.8, 0.8));

    Report report =
        LogDiagnostics.diagnose(log.read(), List.of(new View3dChecks.ElevatorPosesMatchHeight()));

    assertEquals(1, report.getErrorCount());
  }

  /** In sim smoke, a layout key the robot never logged is an error, one per missing key. */
  @Test
  public void layoutKeysMissingFromTheLogAreErrors() throws IOException {
    TestLog log = new TestLog();
    log.text(SimSmokeKeys.AUTO, 1.0, "Elevator Demo");
    log.poses(POSES, 1.0, poses(0, 0));
    LogFile read = log.read();

    Report report = LogDiagnostics.diagnose(read, List.of(new View3dChecks.LayoutKeysAreLogged()));

    // Missing: the robot pose (read by the robot and the ghost) and the goal poses.
    assertEquals(3, report.getErrorCount());
  }
}
