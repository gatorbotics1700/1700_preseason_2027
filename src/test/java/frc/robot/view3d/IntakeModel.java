package frc.robot.view3d;

// TODO - TEMP CODE COPIED FROM SIMON
// Follows ElevatorModel from the elevator prototype (PR #8), changed for the intake arm.

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.view3d.Glb.Box;
import frc.robot.view3d.Glb.Part;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The intake robot's 3D model for AdvantageScope, built from boxes (we have no CAD yet). The arm is
 * drawn retracted, standing straight up from its pivot; the logged pose tips it from there.
 *
 * <p>Files (in ascope_assets/Robot_GatorboticsIntake/), written by IntakeModelTest:
 *
 * <ul>
 *   <li>model.glb: chassis and bumpers.
 *   <li>model_0.glb: the intake arm and its rollers, moved by ComponentPoses[0].
 *   <li>config.json: the robot's name and its one moving part.
 * </ul>
 */
public final class IntakeModel {

  /** Where the model files live. AdvantageScope loads this folder as custom assets. */
  public static final Path FOLDER = Path.of("ascope_assets", "Robot_GatorboticsIntake");

  /** The robot's name in AdvantageScope's model list. */
  public static final String ROBOT_NAME = "Gatorbotics Intake Bot";

  // Colors: red, green, blue, opacity.
  private static final double[] GREEN = {0.05, 0.45, 0.15, 1.0}; // chassis
  private static final double[] DARK = {0.08, 0.08, 0.08, 1.0}; // bumpers
  private static final double[] ORANGE = {0.95, 0.45, 0.05, 1.0}; // arm
  private static final double[] BLUE = {0.10, 0.30, 0.85, 1.0}; // rollers

  // TODO - PLACEHOLDER. Replace with actual values when we have them from CAD
  // Only used to draw the robot; the physics does not read these.
  private static final double CHASSIS_LENGTH_METERS = 0.70;
  private static final double CHASSIS_WIDTH_METERS = 0.70;
  private static final double CHASSIS_HEIGHT_METERS = 0.10;
  private static final double CHASSIS_BOTTOM_Z_METERS = 0.05;
  private static final double BUMPER_THICKNESS_METERS = 0.08;
  private static final double BUMPER_HEIGHT_METERS = 0.12;
  private static final double ARM_WIDTH_METERS = 0.60; // side to side, across the robot
  private static final double ARM_PLATE_THICKNESS_METERS = 0.02;
  private static final double ROLLER_SIZE_METERS = 0.06;

  private static final int UPRIGHT_DEGREES = 90;

  private IntakeModel() {}

  /** Every model file, by file name, as bytes. Always in the same order. */
  public static Map<String, byte[]> files() throws IOException {
    Map<String, byte[]> files = new LinkedHashMap<>();
    files.put("model.glb", Glb.write(List.of(chassis(), bumpers())));
    files.put("model_0.glb", Glb.write(List.of(armPlates(), roller())));
    files.put("config.json", configJson());
    return files;
  }

  /** The chassis: one flat box. */
  private static Part chassis() {
    double centerZ = CHASSIS_BOTTOM_Z_METERS + CHASSIS_HEIGHT_METERS / 2;
    Box box =
        Box.centered(
            0, 0, centerZ, CHASSIS_LENGTH_METERS, CHASSIS_WIDTH_METERS, CHASSIS_HEIGHT_METERS);
    return new Part(GREEN, List.of(box));
  }

  /** Four bumpers around the chassis. */
  private static Part bumpers() {
    double length = CHASSIS_LENGTH_METERS;
    double width = CHASSIS_WIDTH_METERS;
    double height = BUMPER_HEIGHT_METERS;
    double centerZ = CHASSIS_BOTTOM_Z_METERS + CHASSIS_HEIGHT_METERS / 2;
    double thick = BUMPER_THICKNESS_METERS;
    List<Box> boxes = new ArrayList<>();
    boxes.add(Box.centered(length / 2 + thick / 2, 0, centerZ, thick, width + 2 * thick, height));
    boxes.add(Box.centered(-length / 2 - thick / 2, 0, centerZ, thick, width + 2 * thick, height));
    boxes.add(Box.centered(0, width / 2 + thick / 2, centerZ, length, thick, height));
    boxes.add(Box.centered(0, -width / 2 - thick / 2, centerZ, length, thick, height));
    return new Part(DARK, boxes);
  }

  /** The two arm plates, one on each side, standing straight up from the pivot. */
  private static Part armPlates() {
    double x = IntakeConstants.PIVOT_X_METERS;
    double centerZ = IntakeConstants.PIVOT_Z_METERS + IntakeConstants.ARM_LENGTH_METERS / 2;
    double sideY = ARM_WIDTH_METERS / 2;
    double thick = ARM_PLATE_THICKNESS_METERS;
    double length = IntakeConstants.ARM_LENGTH_METERS;
    List<Box> boxes = new ArrayList<>();
    boxes.add(Box.centered(x, sideY, centerZ, thick, thick, length));
    boxes.add(Box.centered(x, -sideY, centerZ, thick, thick, length));
    return new Part(ORANGE, boxes);
  }

  /** The roller bar across the top (far end) of the arm. */
  private static Part roller() {
    double x = IntakeConstants.PIVOT_X_METERS;
    double z = IntakeConstants.PIVOT_Z_METERS + IntakeConstants.ARM_LENGTH_METERS;
    double size = ROLLER_SIZE_METERS;
    Box box = Box.centered(x, 0, z, size, ARM_WIDTH_METERS, size);
    return new Part(BLUE, List.of(box));
  }

  /** AdvantageScope's robot config: the name, which way is up, and one entry per moving part. */
  private static byte[] configJson() throws IOException {
    ObjectMapper json = new ObjectMapper();
    json.enable(SerializationFeature.INDENT_OUTPUT);
    ObjectNode config = json.createObjectNode();
    config.put("name", ROBOT_NAME);
    config.put("isFTC", false);
    upright(config.putArray("rotations"));
    atOrigin(config.putArray("position"));
    config.putArray("cameras");
    ArrayNode components = config.putArray("components");
    int movingParts = IntakeSubsystem.componentPoses(0).length;
    for (int i = 0; i < movingParts; i++) {
      ObjectNode component = components.addObject();
      upright(component.putArray("zeroedRotations"));
      atOrigin(component.putArray("zeroedPosition"));
    }
    String text = json.writeValueAsString(config) + "\n";
    return text.getBytes(StandardCharsets.UTF_8);
  }

  /** Turns the model 90 degrees about x, because glTF is y-up and AdvantageScope is z-up. */
  private static void upright(ArrayNode rotations) {
    ObjectNode turn = rotations.addObject();
    turn.put("axis", "x");
    turn.put("degrees", UPRIGHT_DEGREES);
  }

  /** Fills a position with 0, 0, 0: the model sits where it was drawn. */
  private static void atOrigin(ArrayNode position) {
    position.add(0);
    position.add(0);
    position.add(0);
  }
}
