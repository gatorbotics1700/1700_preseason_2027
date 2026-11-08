package frc.robot.view3d;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.diagnostics.LogCheck;
import frc.robot.diagnostics.LogFile;
import frc.robot.diagnostics.LogFile.PosesSample;
import frc.robot.diagnostics.Report;
import frc.robot.diagnostics.SimSmokeKeys;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Log checks that keep the 3D view honest: what the robot logged is what AdvantageScope will draw.
 * LogDiagnostics runs them with the other checks.
 */
public final class View3dChecks {

  /** The team's AdvantageScope layout, which the 3D view tab reads its keys from. */
  public static final Path LAYOUT =
      Path.of("src", "main", "deploy", "AdvantageScope-Layout-Elevator.json");

  // AdvantageScope's number for a 3D field tab in a layout file.
  static final int FIELD_3D_TAB = 3;

  private static final double DELTA = 1e-6;

  private View3dChecks() {}

  /** Every 3D view check. */
  public static List<LogCheck> all() {
    List<LogCheck> checks = new ArrayList<>();
    checks.add(new ElevatorPosesMatchHeight());
    checks.add(new LayoutKeysAreLogged());
    return checks;
  }

  /**
   * Every logged ComponentPoses matches the logged height: stage 1 at height / RIGGING_RATIO, the
   * carriage at the height, straight up. Then the 3D model moves exactly as the code thinks.
   */
  static class ElevatorPosesMatchHeight implements LogCheck {
    private static final String POSITION = "/Mech/Elevator/IO/PositionMeters";
    private static final String POSES = "/RealOutputs/Mech/Elevator/ComponentPoses";

    public String name() {
      return "3D view: elevator poses match its height";
    }

    public String[] requiredKeys() {
      return new String[] {POSITION, POSES};
    }

    public void check(LogFile log, Report report) {
      int wrong = 0;
      List<PosesSample> samples = log.poses(POSES);
      for (PosesSample sample : samples) {
        double heightMeters = log.numberAt(POSITION, sample.timeSeconds);
        if (!posesMatch(sample.value, heightMeters)) {
          wrong++;
        }
      }
      if (wrong > 0) {
        report.error(wrong + " of " + samples.size() + " ComponentPoses disagree with the height");
      }
    }

    /** True if the poses are where the cascade puts them for this height. */
    private static boolean posesMatch(Pose3d[] poses, double heightMeters) {
      if (poses.length != 2 || Double.isNaN(heightMeters)) {
        return false;
      }
      double stage1Meters = heightMeters / ElevatorConstants.RIGGING_RATIO;
      boolean heightsRight =
          Math.abs(poses[0].getZ() - stage1Meters) < DELTA
              && Math.abs(poses[1].getZ() - heightMeters) < DELTA;
      boolean straightUp = true;
      for (Pose3d pose : poses) {
        straightUp &= Math.abs(pose.getX()) < DELTA && Math.abs(pose.getY()) < DELTA;
        straightUp &= Math.abs(pose.getRotation().getAngle()) < DELTA;
      }
      return heightsRight && straightUp;
    }
  }

  /**
   * Simulator only: every key the layout's 3D tab reads is in the log. A key the robot never logs
   * makes AdvantageScope quietly draw nothing (this once hid the whole robot).
   */
  static class LayoutKeysAreLogged implements LogCheck {
    public String name() {
      return "3D view: layout keys are logged";
    }

    public String[] requiredKeys() {
      return new String[] {SimSmokeKeys.AUTO};
    }

    public void check(LogFile log, Report report) {
      try {
        for (JsonNode source : field3dSources()) {
          String key = source.get("logKey").asText();
          if (!log.has(key)) {
            report.error("the layout reads " + key + ", which the robot never logs");
          }
        }
      } catch (IOException e) {
        report.error("can't read " + LAYOUT + ": " + e.getMessage());
      }
    }
  }

  /** The things the layout's first 3D tab draws: robot, moving parts, ghost, in that order. */
  static List<JsonNode> field3dSources() throws IOException {
    JsonNode layout = new ObjectMapper().readTree(LAYOUT.toFile());
    for (JsonNode hub : layout.get("hubs")) {
      for (JsonNode tab : hub.get("state").get("tabs").get("tabs")) {
        if (tab.get("type").asInt() == FIELD_3D_TAB) {
          List<JsonNode> sources = new ArrayList<>();
          for (JsonNode source : tab.get("controller").get("sources")) {
            sources.add(source);
          }
          return sources;
        }
      }
    }
    throw new IOException("no 3D field tab in " + LAYOUT);
  }
}
