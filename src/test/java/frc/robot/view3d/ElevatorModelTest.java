package frc.robot.view3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.mech.ElevatorSubsystem;
import frc.robot.view3d.Glb.Box;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * Checks the elevator's 3D model files match ElevatorConstants and the logged poses. After changing
 * a size, the tests rewrite the stale files and fail once; run again and commit.
 */
public class ElevatorModelTest {

  private static final double TOLERANCE_METERS = 1e-4; // model files store float32
  private static final double DELTA = 1e-9;

  /** The model files on disk match what ElevatorModel builds, and stale files are rewritten. */
  @Test
  public void modelFilesAreUpToDate() throws IOException {
    List<String> rebuilt = new ArrayList<>();
    Files.createDirectories(ElevatorModel.FOLDER);
    for (Map.Entry<String, byte[]> file : ElevatorModel.files().entrySet()) {
      Path path = ElevatorModel.FOLDER.resolve(file.getKey());
      if (!Files.exists(path) || !Arrays.equals(Files.readAllBytes(path), file.getValue())) {
        Files.write(path, file.getValue());
        rebuilt.add(file.getKey());
      }
    }
    if (!rebuilt.isEmpty()) {
      fail(
          "Rebuilt "
              + rebuilt
              + " in "
              + ElevatorModel.FOLDER
              + ". Run the tests again and commit them.");
    }
  }

  /** The model folder holds only the files ElevatorModel builds. */
  @Test
  public void noOtherFilesInModelFolder() throws IOException {
    for (File file : ElevatorModel.FOLDER.toFile().listFiles()) {
      assertTrue(ElevatorModel.files().containsKey(file.getName()), "unexpected file " + file);
    }
  }

  /** The config names the robot and has one moving part for each pose the subsystem logs. */
  @Test
  public void configHasOnePartPerLoggedPose() throws IOException {
    File configFile = ElevatorModel.FOLDER.resolve("config.json").toFile();
    JsonNode config = new ObjectMapper().readTree(configFile);
    int loggedPoses = ElevatorSubsystem.componentPoses(0).length;

    assertEquals(ElevatorModel.ROBOT_NAME, config.get("name").asText());
    assertEquals(loggedPoses, config.get("components").size());
    for (int i = 0; i < loggedPoses; i++) {
      assertTrue(
          Files.isRegularFile(ElevatorModel.FOLDER.resolve("model_" + i + ".glb")), "model_" + i);
    }
  }

  /** The poses follow the cascade: stage 1 rises half as far as the carriage, straight up. */
  @Test
  public void componentPosesFollowCascadeRigging() {
    for (double heightMeters : new double[] {0.0, 0.37, 1.20}) {
      Pose3d[] poses = ElevatorSubsystem.componentPoses(heightMeters);
      assertEquals(2, poses.length);
      assertEquals(heightMeters / 2, poses[0].getZ(), DELTA, "stage 1");
      assertEquals(heightMeters, poses[1].getZ(), DELTA, "carriage");
      for (Pose3d pose : poses) {
        assertEquals(0, pose.getX(), DELTA);
        assertEquals(0, pose.getY(), DELTA);
        assertEquals(0, pose.getRotation().getAngle(), DELTA);
      }
    }
  }

  /** The fixed stage in the model is as tall as the constants say. */
  @Test
  public void fixedStageMatchesConstants() throws IOException {
    Box base = Glb.readBounds(ElevatorModel.FOLDER.resolve("model.glb"));
    double topZ = ElevatorConstants.BASE_BOTTOM_Z_METERS + ElevatorConstants.BASE_HEIGHT_METERS;
    assertEquals(topZ, base.maxZMeters, TOLERANCE_METERS, "base top");
  }

  /** Stage 1 in the model matches the constants. */
  @Test
  public void stage1MatchesConstants() throws IOException {
    Box stage1 = Glb.readBounds(ElevatorModel.FOLDER.resolve("model_0.glb"));
    double bottomZ = ElevatorConstants.STAGE_1_BOTTOM_Z_METERS;
    double halfWidth =
        (ElevatorConstants.STAGE_1_RAIL_SPACING_METERS + ElevatorConstants.RAIL_TUBE_Y_METERS) / 2;
    assertEquals(bottomZ, stage1.minZMeters, TOLERANCE_METERS);
    assertEquals(
        bottomZ + ElevatorConstants.STAGE_1_HEIGHT_METERS, stage1.maxZMeters, TOLERANCE_METERS);
    assertEquals(halfWidth, stage1.maxYMeters, TOLERANCE_METERS);
    assertEquals(-halfWidth, stage1.minYMeters, TOLERANCE_METERS);
  }

  /** The carriage in the model matches the constants. */
  @Test
  public void carriageMatchesConstants() throws IOException {
    Box carriage = Glb.readBounds(ElevatorModel.FOLDER.resolve("model_1.glb"));
    double bottomZ = ElevatorConstants.CARRIAGE_BOTTOM_Z_METERS;
    assertEquals(bottomZ, carriage.minZMeters, TOLERANCE_METERS);
    assertEquals(
        bottomZ + ElevatorConstants.CARRIAGE_HEIGHT_METERS, carriage.maxZMeters, TOLERANCE_METERS);
    assertEquals(
        ElevatorConstants.CARRIAGE_WIDTH_METERS / 2, carriage.maxYMeters, TOLERANCE_METERS);
  }

  /** Each stage sits fully behind the one it rides in, so no parts collide anywhere in travel. */
  @Test
  public void movingPartsClearEachOtherThroughFullTravel() throws IOException {
    Box base = Glb.readBounds(ElevatorModel.FOLDER.resolve("model.glb"));
    Box stage1 = Glb.readBounds(ElevatorModel.FOLDER.resolve("model_0.glb"));
    Box carriage = Glb.readBounds(ElevatorModel.FOLDER.resolve("model_1.glb"));
    double baseRailBackX =
        ElevatorConstants.ELEVATOR_X_METERS - ElevatorConstants.RAIL_TUBE_X_METERS / 2;
    assertTrue(
        stage1.maxXMeters <= baseRailBackX + TOLERANCE_METERS, "stage 1 hits the fixed stage");
    assertTrue(
        carriage.maxXMeters <= stage1.minXMeters + TOLERANCE_METERS, "carriage hits stage 1");
    assertTrue(
        base.maxXMeters >= ElevatorConstants.ELEVATOR_X_METERS, "fixed stage missing from model");
  }
}
