package frc.robot.view3d;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.mech.ElevatorSubsystem;
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
 * The elevator robot's 3D model for AdvantageScope, built from boxes sized by ElevatorConstants.
 * Each part is drawn with the elevator fully down, so each logged pose is how far it has risen.
 *
 * <p>Files (in ascope_assets/Robot_GatorboticsElevator/):
 *
 * <ul>
 *   <li>model.glb: chassis, bumpers and the fixed stage.
 *   <li>model_0.glb: stage 1, moved by ComponentPoses[0].
 *   <li>model_1.glb: the carriage, moved by ComponentPoses[1].
 *   <li>config.json: the robot's name and its moving parts.
 * </ul>
 */
public final class ElevatorModel {

  /** Where the model files live. AdvantageScope loads this folder as custom assets. */
  public static final Path FOLDER = Path.of("ascope_assets", "Robot_GatorboticsElevator");

  /** The robot's name in AdvantageScope's model list. */
  public static final String ROBOT_NAME = "Gatorbotics Elevator Bot";

  // Colors: red, green, blue, opacity.
  private static final double[] GREEN = {0.05, 0.45, 0.15, 1.0}; // chassis
  private static final double[] DARK = {0.08, 0.08, 0.08, 1.0}; // bumpers
  private static final double[] ORANGE = {0.95, 0.45, 0.05, 1.0}; // fixed stage
  private static final double[] SILVER = {0.75, 0.75, 0.78, 1.0}; // stage 1
  private static final double[] BLUE = {0.10, 0.30, 0.85, 1.0}; // carriage

  private static final double BUMPER_THICKNESS_METERS = 0.08;
  // Bumpers stand a little taller than the chassis.
  private static final double BUMPER_HEIGHT_FACTOR = 1.2;
  private static final int UPRIGHT_DEGREES = 90;
  private static final double CARRIAGE_PLATE_THICKNESS_METERS = 0.02;
  // A shelf on the carriage makes its motion easy to see.
  private static final double SHELF_DEPTH_METERS = 0.24;
  private static final double SHELF_THICKNESS_METERS = 0.02;

  private ElevatorModel() {}

  /** Every model file, by file name, as bytes. Always in the same order. */
  public static Map<String, byte[]> files() throws IOException {
    Map<String, byte[]> files = new LinkedHashMap<>();
    files.put("model.glb", Glb.write(List.of(chassis(), bumpers(), fixedStage())));
    files.put("model_0.glb", Glb.write(List.of(stage1())));
    files.put("model_1.glb", Glb.write(List.of(carriage())));
    files.put("config.json", configJson());
    return files;
  }

  /** The chassis: one flat box. */
  private static Part chassis() {
    double centerZ =
        ElevatorConstants.CHASSIS_BOTTOM_Z_METERS + ElevatorConstants.CHASSIS_HEIGHT_METERS / 2;
    Box box =
        Box.centered(
            0,
            0,
            centerZ,
            ElevatorConstants.CHASSIS_LENGTH_METERS,
            ElevatorConstants.CHASSIS_WIDTH_METERS,
            ElevatorConstants.CHASSIS_HEIGHT_METERS);
    return new Part(GREEN, List.of(box));
  }

  /** Four bumpers around the chassis. */
  private static Part bumpers() {
    double length = ElevatorConstants.CHASSIS_LENGTH_METERS;
    double width = ElevatorConstants.CHASSIS_WIDTH_METERS;
    double height = ElevatorConstants.CHASSIS_HEIGHT_METERS * BUMPER_HEIGHT_FACTOR;
    double centerZ =
        ElevatorConstants.CHASSIS_BOTTOM_Z_METERS + ElevatorConstants.CHASSIS_HEIGHT_METERS / 2;
    double thick = BUMPER_THICKNESS_METERS;
    List<Box> boxes = new ArrayList<>();
    boxes.add(Box.centered(length / 2 + thick / 2, 0, centerZ, thick, width + 2 * thick, height));
    boxes.add(Box.centered(-length / 2 - thick / 2, 0, centerZ, thick, width + 2 * thick, height));
    boxes.add(Box.centered(0, width / 2 + thick / 2, centerZ, length, thick, height));
    boxes.add(Box.centered(0, -width / 2 - thick / 2, centerZ, length, thick, height));
    return new Part(DARK, boxes);
  }

  /** The fixed stage: two rails and two crossbars, at the front of the elevator. */
  private static Part fixedStage() {
    List<Box> boxes =
        rails(
            ElevatorConstants.ELEVATOR_X_METERS,
            ElevatorConstants.BASE_RAIL_SPACING_METERS,
            ElevatorConstants.BASE_BOTTOM_Z_METERS,
            ElevatorConstants.BASE_HEIGHT_METERS);
    return new Part(ORANGE, boxes);
  }

  /** Stage 1: rails just behind the fixed stage's rails. */
  private static Part stage1() {
    List<Box> boxes =
        rails(
            stage1X(),
            ElevatorConstants.STAGE_1_RAIL_SPACING_METERS,
            ElevatorConstants.STAGE_1_BOTTOM_Z_METERS,
            ElevatorConstants.STAGE_1_HEIGHT_METERS);
    return new Part(SILVER, boxes);
  }

  /** The carriage: a plate just behind stage 1, with a shelf on its back. */
  private static Part carriage() {
    double plateX = stage1X() - ElevatorConstants.RAIL_TUBE_X_METERS;
    double bottomZ = ElevatorConstants.CARRIAGE_BOTTOM_Z_METERS;
    double height = ElevatorConstants.CARRIAGE_HEIGHT_METERS;
    double width = ElevatorConstants.CARRIAGE_WIDTH_METERS;
    double shelfX = plateX - CARRIAGE_PLATE_THICKNESS_METERS / 2 - SHELF_DEPTH_METERS / 2;
    double shelfZ = bottomZ + SHELF_THICKNESS_METERS / 2;
    Box plate =
        Box.centered(
            plateX, 0, bottomZ + height / 2, CARRIAGE_PLATE_THICKNESS_METERS, width, height);
    Box shelf = Box.centered(shelfX, 0, shelfZ, SHELF_DEPTH_METERS, width, SHELF_THICKNESS_METERS);
    return new Part(BLUE, List.of(plate, shelf));
  }

  /** Where stage 1's rails are along x: one tube behind the fixed stage. */
  private static double stage1X() {
    return ElevatorConstants.ELEVATOR_X_METERS - ElevatorConstants.RAIL_TUBE_X_METERS;
  }

  /** Two upright rails (2x1 tube) plus a crossbar at the bottom and the top, centered on y = 0. */
  private static List<Box> rails(double x, double spacing, double bottomZ, double height) {
    double tubeX = ElevatorConstants.RAIL_TUBE_X_METERS;
    double tubeY = ElevatorConstants.RAIL_TUBE_Y_METERS;
    double centerZ = bottomZ + height / 2;
    double crossbarLength = spacing + tubeY;
    List<Box> boxes = new ArrayList<>();
    boxes.add(Box.centered(x, spacing / 2, centerZ, tubeX, tubeY, height));
    boxes.add(Box.centered(x, -spacing / 2, centerZ, tubeX, tubeY, height));
    boxes.add(Box.centered(x, 0, bottomZ + tubeY / 2, tubeX, crossbarLength, tubeY));
    boxes.add(Box.centered(x, 0, bottomZ + height - tubeY / 2, tubeX, crossbarLength, tubeY));
    return boxes;
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
    // Same order as ComponentPoses: stage 1, carriage.
    int movingParts = ElevatorSubsystem.componentPoses(0).length;
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
