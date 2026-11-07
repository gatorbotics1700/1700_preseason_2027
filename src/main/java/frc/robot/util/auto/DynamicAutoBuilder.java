package frc.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.mech.shooter.ShooterSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class DynamicAutoBuilder {

  private final IntakeSubsystem intakeSubsystem;
  private final Drive drive;
  private final HoodSubsystem hoodSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  public DynamicAutoBuilder(
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.intakeSubsystem = intakeSubsystem;
    this.drive = drive;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.robotPose = robotPose;
    this.chassisSpeeds = chassisSpeeds;
  }

  private String convertFromLocation(String location) {
    return location;
  }

  private boolean isInvalid(String value) {
    return value == null || value.equals("None");
  }

  private String buildPathName(String alliance, String from, String to) {
    // Need all three values to build a path name
    if (isInvalid(alliance) || isInvalid(from) || isInvalid(to)) {
      return null;
    }

    String convertedFrom = convertFromLocation(from);
    return alliance + " " + convertedFrom + " to " + to;
  }

  private boolean isInAllianceZone() {
    double x = robotPose.get().getX();
    return x <= FieldCoordinates.BLUE_BUMP_AND_TRENCH_X
        || x >= FieldCoordinates.RED_BUMP_AND_TRENCH_X;
  }

  // /** Creates shooting command that shoots only when in alliance zone. */ //TODO: check what we
  // want to do with this
  // private Command createShootingWithZoneCheck() {
  //   return Commands.run(() -> shooterSubsystem.setShouldShoot(isInAllianceZone()))
  //       .alongWith(
  //           ShootingCommands.ShootOnTheMoveCommand(
  //               shooterSubsystem,
  //               hoodSubsystem,
  //               hopperFloorSubsystem,
  //               turretSubsystem,
  //               robotPose,
  //               chassisSpeeds));
  // }

  private Command getActionForDestination(String destination) {
    if (destination == null || destination.equals("None")) {
      return Commands.none();
    }

    if (destination.startsWith("Fuel Pile") && RobotBase.isReal()) {
      return new HoodCommands.HoodRetractCommand(hoodSubsystem)
          .onlyWhile(() -> !isInAllianceZone());
    }

    return Commands.none();
  }

  private Command loadPathCommand(String alliance, String from, String to) {
    String pathName = buildPathName(alliance, from, to);
    if (pathName == null) {
      System.out.println("  Could not build path name for: " + alliance + " " + from + " to " + to);
      return Commands.none();
    }

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      System.out.println("  Loaded path: " + pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("  Path not available: " + pathName + " - " + e.getMessage());
      return Commands.none();
    }
  }

  private Command loadDepotToTowerPath(String alliance, String depotLocation) {
    // All depots go to Tower Left based on existing path naming
    String pathName = alliance + " " + depotLocation + " to Tower Left";

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      System.out.println("  Loaded depot-to-tower path: " + pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("  Depot-to-tower path not available: " + pathName);
      return null;
    }
  }

  public Command buildAuto(
      String alliance, String startPos, String dest1, String dest2, String dest3) {

    System.out.println("============BUILD AUTO============");
    System.out.println("Alliance: " + alliance);
    System.out.println("First Destination: " + dest1);
    System.out.println("Second Destination: " + dest2);
    System.out.println("Third Destination: " + dest3);

    if (alliance == null || alliance.equals("None")) {
      System.out.println("DynamicAutoBuilder: Missing alliance");
      return Commands.none();
    }
    if (startPos == null || startPos.equals("None")) {
      System.out.println("DynamicAutoBuilder: Missing start position");
      return Commands.none();
    }

    boolean hasDestination = dest1 != null && !dest1.equals("None");
    if (!hasDestination) {
      System.out.println("DynamicAutoBuilder: No destinations selected");
      return Commands.none();
    }

    System.out.println(
        "DynamicAutoBuilder: Building auto - "
            + alliance
            + " "
            + startPos
            + " -> "
            + dest1
            + " -> "
            + dest2
            + " -> "
            + dest3);

    List<Command> commandSequence = new ArrayList<>();
    String currentLocation = startPos;

    // Build the path sequence (without climb)
    List<Command> pathSequence = new ArrayList<>();

    if (dest1 != null && !dest1.equals("None")) {
      Command firstPath = loadPathCommand(alliance, currentLocation, dest1);
      Command firstAction = getActionForDestination(dest1);
      pathSequence.add(firstPath.deadlineFor(firstAction));
      currentLocation = dest1;

      if (dest2 != null && !dest2.equals("None")) {
        Command secondPath = loadPathCommand(alliance, currentLocation, dest2);
        Command secondAction = getActionForDestination(dest2);
        pathSequence.add(secondPath.deadlineFor(secondAction));
        currentLocation = dest2;

        if (dest3 != null && !dest3.equals("None")) {
          Command thirdPath = loadPathCommand(alliance, currentLocation, dest3);
          Command thirdAction = getActionForDestination(dest3);
          pathSequence.add(thirdPath.deadlineFor(thirdAction));
          currentLocation = dest3;
        }
      }
    }

    // Note: Homing is handled by Robot.java calling HomeMechanisms() before auto starts
    // Run all paths with intake and shooting running continuously
    if (!pathSequence.isEmpty()) {
      Command allPaths = Commands.sequence(pathSequence.toArray(new Command[0]));
      if (RobotBase.isReal()) {
        // Deploy intake once at start, then run intake and shooting in parallel with paths
        // Deploy runs alongside paths (doesn't block), intake/shooting run throughout
        Command deployAndIntake =
            IntakeCommands.DeployIntake(intakeSubsystem)
                .alongWith(IntakeCommands.RunIntake(intakeSubsystem));
        /*.alongWith(createShootingWithZoneCheck());*/
        // Paths are the deadline - when paths finish, intake/shooting stop (until climb or end)
        commandSequence.add(allPaths.deadlineFor(deployAndIntake));
      } else {
        // In sim, just run the paths without mech
        commandSequence.add(allPaths);
      }
    }

    if (commandSequence.isEmpty()) {
      System.out.println("DynamicAutoBuilder: No paths loaded - running empty auto");
      return Commands.none();
    }

    return Commands.sequence(commandSequence.toArray(new Command[0]));
  }

  public Optional<String> getFirstPathName(
      String alliance, String startPos, String dest1, String dest2, String dest3) {
    if (isInvalid(alliance) || isInvalid(startPos)) {
      return Optional.empty();
    }
    if (dest1 == null || dest1.equals("None")) {
      return Optional.empty();
    }
    String name = buildPathName(alliance, startPos, dest1);
    return name != null ? Optional.of(name) : Optional.empty();
  }
}
