package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.shooter.ShooterSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveUnderTrenchCommand {

  public static Command driveUnderTrench(Drive drive, ShooterSubsystem shooterSubsystem)
      throws IOException, ParseException {
    PathConstraints constraints =
        new PathConstraints(8, 8, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    Pose2d pose = drive.getPose();
    System.out.println("*********************DRIVE UNDER TRENCH COMMAND*******************");

    Command pathToFollow;

    String pathRotation;
    double robotRotationDegrees = pose.getRotation().getDegrees(); // from -180 to 180
    if (-90 < robotRotationDegrees && robotRotationDegrees <= 90) {
      pathRotation = " 0";
    } else {
      pathRotation = " 180";
    }

    if (pose.getX() <= FieldCoordinates.FIELD_CENTER.getX()) { // BLUE
      if (pose.getY() < FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TR A to N" + pathRotation), constraints);
      } else if (pose.getY() > FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TL A to N" + pathRotation), constraints);
        // System.out.println(PathPlannerPath.fromPathFile("B TL A to N" + pathRotation),
        // constraints).toString())
      } else if ((pose.getY() > FieldCoordinates.FIELD_CENTER.getY())
          && (pose.getX() > FieldCoordinates.BLUE_BUMP_AND_TRENCH_X)) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TL N to A" + pathRotation), constraints);
      } else {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TR N to A" + pathRotation), constraints);
      }
    } else { // RED
      if (pose.getY() < FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TL N to A" + pathRotation), constraints);
      } else if (pose.getY() > FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TR N to A" + pathRotation), constraints);
      } else if ((pose.getY() > FieldCoordinates.FIELD_CENTER.getY())
          && (pose.getX() > FieldCoordinates.RED_BUMP_AND_TRENCH_X)) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TR A to N" + pathRotation), constraints);
      } else {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TL A to N" + pathRotation), constraints);
      }
    }
    // System.out.println("drive under trench:"+pathRotation);
    return pathToFollow.withName("DriveUnderTrench");
  }
}
