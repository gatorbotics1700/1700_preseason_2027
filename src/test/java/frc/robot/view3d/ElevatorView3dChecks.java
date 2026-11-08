package frc.robot.view3d;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.diagnostics.LogCheck;
import frc.robot.diagnostics.LogFile;
import frc.robot.diagnostics.LogFile.PosesSample;
import frc.robot.diagnostics.Report;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** Log checks that keep the elevator's 3D view honest. */
public final class ElevatorView3dChecks {

  /** The elevator's AdvantageScope layout. */
  public static final Path LAYOUT =
      Path.of("src", "main", "deploy", "AdvantageScope-Layout-Elevator.json");

  private static final double DELTA = 1e-6;

  private ElevatorView3dChecks() {}

  /** Every elevator 3D view check. */
  public static List<LogCheck> all() {
    List<LogCheck> checks = new ArrayList<>();
    checks.add(new ElevatorPosesMatchHeight());
    checks.add(new View3dChecks.LayoutKeysAreLogged(LAYOUT));
    return checks;
  }

  /** Every logged ComponentPoses matches the logged height, straight up the cascade. */
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
}
