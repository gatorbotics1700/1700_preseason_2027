package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.Constants.ElevatorConstants;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/**
 * The 3D model in ascope_assets/ must match ElevatorConstants, and its components must line up with
 * the poses ElevatorSubsystem logs. Regenerate with tools/elevator/generate_model.py.
 */
public class ElevatorModelTest {
  private static final Path MODEL_FOLDER = Path.of("ascope_assets", "Robot_GatorboticsElevator");
  private static final ObjectMapper JSON_READER = new ObjectMapper();
  private static final double TOLERANCE_METERS = 1e-4; // models are float32

  /** The box a model part fits in, in the robot frame (+z up), converted back from glTF's Y-up. */
  static class Bounds {
    final double minX;
    final double minY;
    final double minZ;
    final double maxX;
    final double maxY;
    final double maxZ;

    Bounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
      this.minX = minX;
      this.minY = minY;
      this.minZ = minZ;
      this.maxX = maxX;
      this.maxY = maxY;
      this.maxZ = maxZ;
    }
  }

  /** Reads a .glb model file and returns the box it fits in. */
  static Bounds bounds(String file) throws IOException {
    ByteBuffer b = ByteBuffer.wrap(Files.readAllBytes(MODEL_FOLDER.resolve(file)));
    b.order(ByteOrder.LITTLE_ENDIAN);
    assertEquals(0x46546C67, b.getInt(0), file + " is not a GLB");
    int jsonLength = b.getInt(12);
    JsonNode gltf =
        JSON_READER.readTree(new String(b.array(), 20, jsonLength, StandardCharsets.UTF_8).trim());
    double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
    double[] hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (JsonNode a : gltf.get("accessors")) {
      if (a.has("min")) {
        for (int i = 0; i < 3; i++) {
          lo[i] = Math.min(lo[i], a.get("min").get(i).asDouble());
          hi[i] = Math.max(hi[i], a.get("max").get(i).asDouble());
        }
      }
    }
    // glTF (x, y, z) = robot (x, z, -y)
    return new Bounds(lo[0], -hi[2], lo[1], hi[0], -lo[2], hi[1]);
  }

  /** The model has one moving part for each pose the subsystem logs. */
  @Test
  public void configDeclaresOneComponentPerLoggedPose() throws IOException {
    JsonNode config = JSON_READER.readTree(MODEL_FOLDER.resolve("config.json").toFile());
    assertEquals("Gatorbotics Elevator Bot", config.get("name").asText());
    int poses = ElevatorSubsystem.componentPoses(0).length;
    assertEquals(poses, config.get("components").size());
    for (int i = 0; i < poses; i++) {
      assertTrue(Files.isRegularFile(MODEL_FOLDER.resolve("model_" + i + ".glb")), "model_" + i);
    }
    assertTrue(Files.isRegularFile(MODEL_FOLDER.resolve("model.glb")));
  }

  /** The fixed stage in the model is as tall as the constants say. */
  @Test
  public void fixedStageMatchesConstants() throws IOException {
    Bounds b = bounds("model.glb");
    assertEquals(
        ElevatorConstants.BASE_BOTTOM_Z_METERS + ElevatorConstants.BASE_HEIGHT_METERS,
        b.maxZ,
        TOLERANCE_METERS,
        "base top");
  }

  /** Stage 1 in the model matches the constants. */
  @Test
  public void stage1MatchesConstants() throws IOException {
    Bounds b = bounds("model_0.glb");
    assertEquals(ElevatorConstants.STAGE_1_BOTTOM_Z_METERS, b.minZ, TOLERANCE_METERS);
    assertEquals(
        ElevatorConstants.STAGE_1_BOTTOM_Z_METERS + ElevatorConstants.STAGE_1_HEIGHT_METERS,
        b.maxZ,
        TOLERANCE_METERS);
    double halfWidth =
        (ElevatorConstants.STAGE_1_RAIL_SPACING_METERS + ElevatorConstants.RAIL_TUBE_Y_METERS) / 2;
    assertEquals(halfWidth, b.maxY, TOLERANCE_METERS);
    assertEquals(-halfWidth, b.minY, TOLERANCE_METERS);
  }

  /** The carriage in the model matches the constants. */
  @Test
  public void carriageMatchesConstants() throws IOException {
    Bounds b = bounds("model_1.glb");
    assertEquals(ElevatorConstants.CARRIAGE_BOTTOM_Z_METERS, b.minZ, TOLERANCE_METERS);
    assertEquals(
        ElevatorConstants.CARRIAGE_BOTTOM_Z_METERS + ElevatorConstants.CARRIAGE_HEIGHT_METERS,
        b.maxZ,
        TOLERANCE_METERS);
    assertEquals(ElevatorConstants.CARRIAGE_WIDTH_METERS / 2, b.maxY, TOLERANCE_METERS);
  }

  /** No moving part hits another anywhere in the travel. */
  @Test
  public void movingPartsClearEachOtherThroughFullTravel() throws IOException {
    // Stages slide vertically, so parts at different heights only collide if their x ranges
    // overlap. Each moving stage must sit entirely behind the one it rides in.
    Bounds base = bounds("model.glb");
    Bounds stage1 = bounds("model_0.glb");
    Bounds carriage = bounds("model_1.glb");
    double baseRailMinX =
        ElevatorConstants.ELEVATOR_X_METERS - ElevatorConstants.RAIL_TUBE_X_METERS / 2;
    assertTrue(
        stage1.maxX <= baseRailMinX + TOLERANCE_METERS, "stage 1 intersects fixed stage rails");
    assertTrue(
        carriage.maxX <= stage1.minX + TOLERANCE_METERS, "carriage intersects stage 1 rails");
    assertTrue(
        base.maxX >= ElevatorConstants.ELEVATOR_X_METERS, "fixed stage missing from base model");
  }
}
