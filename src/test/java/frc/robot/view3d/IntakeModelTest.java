package frc.robot.view3d;

// TODO - TEMP CODE COPIED FROM SIMON
// Follows ElevatorModelTest from the elevator prototype (PR #8), changed for the intake arm.

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.mech.IntakeSubsystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Keeps the intake's 3D model files in step with the code, and checks the arm's logged pose. After
 * changing a size, the first test rewrites the files and fails once; run the tests again.
 */
public class IntakeModelTest {

  private static final double DELTA = 1e-9;

  /** Where a point on the arm, drawn straight up, ends up after the logged pose moves it. */
  private static Translation3d moved(Pose3d pose, double x, double z) {
    Translation3d drawn = new Translation3d(x, 0, z);
    return drawn.rotateBy(pose.getRotation()).plus(pose.getTranslation());
  }

  /** The model files on disk match what IntakeModel builds; stale files are rewritten. */
  @Test
  public void modelFilesAreUpToDate() throws IOException {
    List<String> rebuilt = ModelFiles.rewriteStale(IntakeModel.FOLDER, IntakeModel.files());
    assertEquals(List.of(), rebuilt, "rebuilt in " + IntakeModel.FOLDER + ": run the tests again");
  }

  /** The model folder holds only the files IntakeModel builds. */
  @Test
  public void noOtherFilesInModelFolder() throws IOException {
    ModelFiles.rewriteStale(IntakeModel.FOLDER, IntakeModel.files());
    assertEquals(List.of(), ModelFiles.extraFiles(IntakeModel.FOLDER, IntakeModel.files()));
  }

  /** The config names the robot and has one moving part for each pose the intake logs. */
  @Test
  public void configHasOnePartPerLoggedPose() throws IOException {
    ModelFiles.rewriteStale(IntakeModel.FOLDER, IntakeModel.files());
    File configFile = IntakeModel.FOLDER.resolve("config.json").toFile();
    JsonNode config = new ObjectMapper().readTree(configFile);
    int loggedPoses = IntakeSubsystem.componentPoses(0).length;

    assertEquals(IntakeModel.ROBOT_NAME, config.get("name").asText());
    assertEquals(loggedPoses, config.get("components").size());
    for (int i = 0; i < loggedPoses; i++) {
      assertTrue(
          Files.isRegularFile(IntakeModel.FOLDER.resolve("model_" + i + ".glb")), "model_" + i);
    }
  }

  /** Retracted (0 degrees), the arm is drawn exactly where the model has it: no move at all. */
  @Test
  public void retractedArmDoesNotMove() {
    Pose3d arm = IntakeSubsystem.componentPoses(0)[0];
    assertEquals(0, arm.getTranslation().getNorm(), DELTA);
    assertEquals(0, arm.getRotation().getAngle(), DELTA);
  }

  /** At 90 degrees, the arm lies flat behind the robot, and the pivot stays where it is. */
  @Test
  public void armAtNinetyDegreesLiesFlatBehindPivot() {
    Pose3d arm = IntakeSubsystem.componentPoses(90)[0];
    // Pivot at (-0.30, 0.20), arm 0.40 long: its tip goes from straight above to straight behind.
    Translation3d pivot = moved(arm, -0.30, 0.20);
    Translation3d tip = moved(arm, -0.30, 0.60);
    assertEquals(-0.30, pivot.getX(), DELTA);
    assertEquals(0.20, pivot.getZ(), DELTA);
    assertEquals(-0.70, tip.getX(), DELTA);
    assertEquals(0.20, tip.getZ(), DELTA);
  }
}
