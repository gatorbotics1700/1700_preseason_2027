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
 * The code and the model read the same numbers, so the 3D view can't drift from the code.
 * ElevatorModelTest rebuilds it on every test run and fails if the saved files are out of date.
 *
 * <p>This is a stand-in until the build team exports real CAD. Then the .glb files come from CAD,
 * and ElevatorModelTest checks them against the constants instead of rebuilding them.
 *
 * <p>Files (in ascope_assets/Robot_GatorboticsElevator/):
 *
 * <ul>
 *   <li>model.glb: chassis, bumpers and the fixed stage. Moves with the robot.
 *   <li>model_0.glb: stage 1. Moved by ComponentPoses[0].
 *   <li>model_1.glb: the carriage. Moved by ComponentPoses[1].
 *   <li>config.json: tells AdvantageScope the robot's name and its moving parts.
 * </ul>
 *
 * <p>Every part is drawn where it sits with the elevator fully down, so each logged pose is just
 * how far that part has risen.
 */
public final class ElevatorModel {

  /** Where the model files live. AdvantageScope loads this folder as custom assets. */
  public static final Path FOLDER = Path.of("ascope_assets", "Robot_GatorboticsElevator");

  /** The robot's name in AdvantageScope's model list. The layout picks the model by this name. */
  public static final String ROBOT_NAME = "Gatorbotics Elevator Bot";

  // Colors: red, green, blue, opacity.
  private static final double[] GREEN = {0.05, 0.45, 0.15, 1.0}; // chassis
  private static final double[] DARK = {0.08, 0.08, 0.08, 1.0}; // bumpers
  private static final double[] ORANGE = {0.95, 0.45, 0.05, 1.0}; // fixed stage
  private static final double[] SILVER = {0.75, 0.75, 0.78, 1.0}; // stage 1
  private static final double[] BLUE = {0.10, 0.30, 0.85, 1.0}; // carriage

  private static final double BUMPER_THICKNESS_METERS = 0.08;
  private static final double CARRIAGE_PLATE_THICKNESS_METERS = 0.02;
  // A shelf on the carriage's back face makes the motion easy to see. It sits away from the rails
  // and their crossbars, so it never passes through them (ElevatorModelTest checks this).
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
    double height = ElevatorConstants.CHASSIS_HEIGHT_METERS * 1.2;
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
    ObjectMapper json = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    ObjectNode config = json.createObjectNode();
    config.put("name", ROBOT_NAME);
    config.put("isFTC", false);
    upright(config.putArray("rotations"));
    config.putArray("position").add(0).add(0).add(0);
    config.putArray("cameras");
    ArrayNode components = config.putArray("components");
    // One component per moving part, in the same order as ComponentPoses: stage 1, carriage.
    int movingParts = ElevatorSubsystem.componentPoses(0).length;
    for (int i = 0; i < movingParts; i++) {
      ObjectNode component = components.addObject();
      upright(component.putArray("zeroedRotations"));
      component.putArray("zeroedPosition").add(0).add(0).add(0);
    }
    String text = json.writeValueAsString(config) + "\n";
    return text.getBytes(StandardCharsets.UTF_8);
  }

  /**
   * The 90 degree turn about x that makes a glTF model (y up) stand up in AdvantageScope (z up).
   */
  private static void upright(ArrayNode rotations) {
    rotations.addObject().put("axis", "x").put("degrees", 90);
  }
}
