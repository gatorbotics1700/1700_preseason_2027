package frc.robot.commands.drive;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.Constants.DriveToRadiusConstants;
import frc.robot.Constants.FieldCoordinates;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;

public class DriveToRadiusCommand extends Command{
    //
    private final Translation2d HUB_CENTER;
    //
    private static final double ORBIT_RADIUS = DriveToRadiusConstants.SHOT_RADIUS;

    private boolean onArc = false;
 
    //
    private static final PathConstraints PATH_CONSTRAINTS =
            new PathConstraints(3.5, 3.0, Math.toRadians(540), Math.toRadians(720));//TODO stole from HubOrbit need to change???
    private final boolean isRedAlliance;
    private final Drive drive;
    private Command pathCommand = null;
    public DriveToRadiusCommand(Drive drive, boolean isRedAlliance) {

        this.drive = drive;
        this.isRedAlliance = isRedAlliance;
        HUB_CENTER = isRedAlliance ? FieldCoordinates.RED_HUB.toTranslation2d() : FieldCoordinates.BLUE_HUB.toTranslation2d();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        onArc = false;
        schedulePathToArc();
        System.out.println("path scheduled");
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        double distToHub = pose.getTranslation().getDistance(HUB_CENTER);
        double distToArc = Math.abs(distToHub - ORBIT_RADIUS);
 
        if (distToArc <= ON_ARC_TOLERANCE) {
            onArc = true;
        }
    }
 
    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null && pathCommand.isScheduled()) {
            pathCommand.cancel();
        }
        DriveCommands.stopDriveCommand(drive);
    }
 
    @Override
    public boolean isFinished() {
        return false; // runs until button is released
    }

    private void schedulePathToArc() {
        Pose2d currentPose = drive.getPose();
        Translation2d target = closestArcPoint(currentPose.getTranslation());
 
        // Heading at the target: face the hub
        Rotation2d targetHeading = headingToHub(target);

        // Path to hub
        pathCommand = AutoBuilder.pathfindToPose(new Pose2d(target, targetHeading), PATH_CONSTRAINTS, 0.0);
        pathCommand.schedule();
    }

    private Translation2d closestArcPoint(Translation2d robotPos) {
        Translation2d toRobot = robotPos.minus(HUB_CENTER); // finds vector of hub to robot
        double angle = Math.atan2(toRobot.getY(), toRobot.getX()); // extracts angle from hub to robot
        if (isRedAlliance) {
            angle = MathUtil.clamp(angle, -Math.PI / 2.0, Math.PI / 2.0);
        } else {
            // Clamp to [π/2, π] ∪ [-π, -π/2] — keep angle on left half
            if (angle > 0) angle = MathUtil.clamp(angle, Math.PI / 2.0, Math.PI);
            else           angle = MathUtil.clamp(angle, -Math.PI, -Math.PI / 2.0);
        }
        // Red Alliance: robot should be on the right half of hub → angles in [-π/2, +π/2].
        // Blue alliance: robot shuld be on left half → angles in [π/2, 3π/2] (or [-π, -π/2] ∪ [π/2, π])
        // if angle out of bounds then it is clamped to closest in bound angle
        // Bounds might need to be adjusted
        return new Translation2d(
                HUB_CENTER.getX() + ORBIT_RADIUS * Math.cos(angle),
                HUB_CENTER.getY() + ORBIT_RADIUS * Math.sin(angle)
        ); 
        // a point on unit circle is (cos(angle),sin(angle),)
        // multiply the coordinates by the orbit radius to scale up to orbit arc
        // then add hub coordinates to scale to correct position on field
        // this gives us ideal robot pose on arc
    }
    private Rotation2d headingToHub(Translation2d pos) {
        Translation2d toHub = HUB_CENTER.minus(pos);
        // vector of robot to hub
        return new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));
        // extract angle from where robot is facing to hub
    }    
    
    

}