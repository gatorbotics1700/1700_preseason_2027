package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
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
class ElevatorModelTest {
  private static final Path DIR = Path.of("ascope_assets", "Robot_GatorboticsElevator");
  private static final ObjectMapper JSON = new ObjectMapper();
  private static final double TOL = 1e-4; // models are float32

  /** Axis-aligned bounds in the robot frame (+z up), converted back from glTF's Y-up. */
  record Bounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {}

  static Bounds bounds(String file) throws IOException {
    ByteBuffer b = ByteBuffer.wrap(Files.readAllBytes(DIR.resolve(file)));
    b.order(ByteOrder.LITTLE_ENDIAN);
    assertEquals(0x46546C67, b.getInt(0), file + " is not a GLB");
    int jsonLength = b.getInt(12);
    JsonNode gltf =
        JSON.readTree(new String(b.array(), 20, jsonLength, StandardCharsets.UTF_8).trim());
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

  @Test
  void configDeclaresOneComponentPerLoggedPose() throws IOException {
    JsonNode config = JSON.readTree(DIR.resolve("config.json").toFile());
    assertEquals("Gatorbotics Elevator Bot", config.get("name").asText());
    int poses = ElevatorSubsystem.componentPoses(0).length;
    assertEquals(poses, config.get("components").size());
    for (int i = 0; i < poses; i++) {
      assertTrue(Files.isRegularFile(DIR.resolve("model_" + i + ".glb")), "model_" + i);
    }
    assertTrue(Files.isRegularFile(DIR.resolve("model.glb")));
  }

  @Test
  void fixedStageMatchesConstants() throws IOException {
    Bounds b = bounds("model.glb");
    assertEquals(BASE_BOTTOM_Z_METERS + BASE_HEIGHT_METERS, b.maxZ(), TOL, "base top");
  }

  @Test
  void stage1MatchesConstants() throws IOException {
    Bounds b = bounds("model_0.glb");
    assertEquals(STAGE_1_BOTTOM_Z_METERS, b.minZ(), TOL);
    assertEquals(STAGE_1_BOTTOM_Z_METERS + STAGE_1_HEIGHT_METERS, b.maxZ(), TOL);
    double halfWidth = (STAGE_1_RAIL_SPACING_METERS + RAIL_TUBE_Y_METERS) / 2;
    assertEquals(halfWidth, b.maxY(), TOL);
    assertEquals(-halfWidth, b.minY(), TOL);
  }

  @Test
  void carriageMatchesConstants() throws IOException {
    Bounds b = bounds("model_1.glb");
    assertEquals(CARRIAGE_BOTTOM_Z_METERS, b.minZ(), TOL);
    assertEquals(CARRIAGE_BOTTOM_Z_METERS + CARRIAGE_HEIGHT_METERS, b.maxZ(), TOL);
    assertEquals(CARRIAGE_WIDTH_METERS / 2, b.maxY(), TOL);
  }

  @Test
  void movingPartsClearEachOtherThroughFullTravel() throws IOException {
    // Stages slide vertically, so parts at different heights only collide if their x ranges
    // overlap. Each moving stage must sit entirely behind the one it rides in.
    Bounds base = bounds("model.glb");
    Bounds stage1 = bounds("model_0.glb");
    Bounds carriage = bounds("model_1.glb");
    double baseRailMinX = ELEVATOR_X_METERS - RAIL_TUBE_X_METERS / 2;
    assertTrue(stage1.maxX() <= baseRailMinX + TOL, "stage 1 intersects fixed stage rails");
    assertTrue(carriage.maxX() <= stage1.minX() + TOL, "carriage intersects stage 1 rails");
    assertTrue(base.maxX() >= ELEVATOR_X_METERS, "fixed stage missing from base model");
  }
}
