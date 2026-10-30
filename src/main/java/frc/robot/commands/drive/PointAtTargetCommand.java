package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Calculations;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PointAtTargetCommand extends Command {
  private static final int REQUIRED_ON_TARGET_CYCLES = 5;
  private final Drive drive;
  private Supplier<Pose2d> drivetrainPose;
  private int onTargetCycles;

  public PointAtTargetCommand(Drive drive, Supplier<Pose2d> drivetrainPose) {
    setName("PointAtTarget");
    this.drive = drive;
    this.drivetrainPose = drivetrainPose;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    onTargetCycles = 0;
  }

  @Override
  public void execute() {
    Logger.recordOutput("PointAtTargetCommand/isRunning", true);
    Translation3d target;

    if (FieldCoordinates.BLUE_BUMP_AND_TRENCH_X <= drivetrainPose.get().getX()
        && drivetrainPose.get().getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
      if (FieldCoordinates.FIELD_CENTER.getY() < drivetrainPose.get().getY()) {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_RIGHT_FUNNELING
                : FieldCoordinates.RED_LEFT_FUNNELING;

      } else {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_LEFT_FUNNELING
                : FieldCoordinates.RED_RIGHT_FUNNELING;
      }

    } else {
      target =
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldCoordinates.BLUE_HUB
              : FieldCoordinates.RED_HUB;
    }

    double deltaX = target.getX() - drive.getPose().getX();
    double deltaY = target.getY() - drive.getPose().getY();
    Rotation2d desiredHubAngle = Calculations.angleToPoint(deltaX, deltaY);

    drive.driveToPose(new Pose2d(drive.getPose().getTranslation(), desiredHubAngle));
    Logger.recordOutput("PointAtHub/current angle", drive.getPose().getRotation().getDegrees());
    Logger.recordOutput("PointAtHub/desired angle", desiredHubAngle.getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("PoinAtTargetCommand/isRunning", false);
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    Rotation2d desiredHubAngle = drive.getDesiredHubAngle();

    boolean atDesiredRotation =
        drive.calculateRotationError(
                drive.getPose().getRotation().getDegrees(), desiredHubAngle.getDegrees())
            == 0.0;
    Logger.recordOutput(
        "PoinAtTargetCommand/error",
        drive.calculateRotationError(
            drive.getPose().getRotation().getDegrees(), desiredHubAngle.getDegrees()));

    onTargetCycles = atDesiredRotation ? onTargetCycles + 1 : 0;
    return onTargetCycles >= REQUIRED_ON_TARGET_CYCLES;
  }
}
