package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldCoordinates;

public class Calculations {

  public static Rotation2d angleToPoint(double deltaX, double deltaY) {
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }

  public static Boolean withinDeadband(
      double one, double two, double deadband, Boolean containsEquals) {
    if (containsEquals) {
      return (Math.abs(one - two) <= deadband);
    } else {
      return (Math.abs(one - two) < deadband);
    }
  }

  public static double distanceToPoseInMeters(Pose2d firstPose, Pose2d secondPose) {
    return Math.sqrt(
        Math.pow(firstPose.getX() - secondPose.getX(), 2)
            + Math.pow(firstPose.getY() - secondPose.getY(), 2));
  }

  public static Pose2d mirrorPoseAcrossAlliance(Pose2d pose) {
    double deltaX = pose.getX() - FieldCoordinates.FIELD_CENTER.getX();
    double deltaRotation = pose.getRotation().getDegrees() - 90;
    return new Pose2d(
        FieldCoordinates.FIELD_CENTER.getX() - deltaX,
        pose.getY(),
        new Rotation2d(Math.toRadians(90 - deltaRotation)));
  }

  public static Pose2d mirrorPoseAcrossXAxis(Pose2d pose) {
    double deltaY = pose.getY() - FieldCoordinates.FIELD_CENTER.getY();
    return new Pose2d(
        pose.getX(),
        FieldCoordinates.FIELD_CENTER.getY() - deltaY,
        pose.getRotation().unaryMinus());
  }
}
