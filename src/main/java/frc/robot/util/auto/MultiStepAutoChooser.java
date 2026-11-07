package frc.robot.util.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.mech.shooter.ShooterSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class MultiStepAutoChooser {
  private static MultiStepAutoChooser instance;

  private final LoggedDashboardChooser<String> allianceChooser;
  private final LoggedDashboardChooser<String> startPosChooser;
  private final DynamicAutoBuilder dynamicAutoBuilder;

  // First destination choosers (location type + side)
  private final LoggedDashboardChooser<String> firstDestinationTypeChooser;
  private final LoggedDashboardChooser<String> firstDestinationSideChooser;

  // Second destination choosers (location type + side)
  private final LoggedDashboardChooser<String> secondDestinationTypeChooser;
  private final LoggedDashboardChooser<String> secondDestinationSideChooser;

  // Third destination choosers (location type + side)
  private final LoggedDashboardChooser<String> thirdDestinationTypeChooser;
  private final LoggedDashboardChooser<String> thirdDestinationSideChooser;

  public MultiStepAutoChooser(
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    instance = this;

    // Create the dynamic auto builder
    this.dynamicAutoBuilder =
        new DynamicAutoBuilder(
            intakeSubsystem,
            drive,
            hoodSubsystem,
            shooterSubsystem,
            turretSubsystem,
            hopperFloorSubsystem,
            robotPose,
            chassisSpeeds);

    // Initialize choosers
    allianceChooser = new LoggedDashboardChooser<>("Auto/Alliance");
    startPosChooser = new LoggedDashboardChooser<>("Auto/Start Position");

    firstDestinationTypeChooser =
        new LoggedDashboardChooser<>("Auto/First Destination/First Destination Type");
    firstDestinationSideChooser =
        new LoggedDashboardChooser<>("Auto/First Destination/First Destination Side");

    secondDestinationTypeChooser =
        new LoggedDashboardChooser<>("Auto/Second Destination/Second Destination Type");
    secondDestinationSideChooser =
        new LoggedDashboardChooser<>("Auto/Second Destination/Second Destination Side");

    thirdDestinationTypeChooser =
        new LoggedDashboardChooser<>("Auto/Third Destination/Third Destination Type");
    thirdDestinationSideChooser =
        new LoggedDashboardChooser<>("Auto/Third Destination/Third Destination Side");

    // Populate alliance chooser with hardcoded values
    allianceChooser.addDefaultOption("None", "None");
    allianceChooser.addOption("Red", "R");
    allianceChooser.addOption("Blue", "B");

    // Populate start position chooser with hardcoded values
    startPosChooser.addDefaultOption("None", "None");
    startPosChooser.addOption("Center", "Center");
    startPosChooser.addOption("Left Far", "LF");
    startPosChooser.addOption("Left Near", "LN");
    startPosChooser.addOption("Right Far", "RF");
    startPosChooser.addOption("Right Near", "RN");

    // Populate first destination type chooser
    firstDestinationTypeChooser.addDefaultOption("None", "None");
    firstDestinationTypeChooser.addOption("Depot", "Depot");
    firstDestinationTypeChooser.addOption("Outpost", "Outpost");
    firstDestinationTypeChooser.addOption("Fuel Pile", "Fuel Pile");

    // Populate first destination side chooser
    firstDestinationSideChooser.addDefaultOption("None", "None");
    firstDestinationSideChooser.addOption("Left", "Left");
    firstDestinationSideChooser.addOption("Right", "Right");
    firstDestinationSideChooser.addOption("Center", "Center");

    // Populate second destination type chooser
    secondDestinationTypeChooser.addDefaultOption("None", "None");
    secondDestinationTypeChooser.addOption("Depot", "Depot");
    secondDestinationTypeChooser.addOption("Outpost", "Outpost");
    secondDestinationTypeChooser.addOption("Fuel Pile", "Fuel Pile");

    // Populate second destination side chooser
    secondDestinationSideChooser.addDefaultOption("None", "None");
    secondDestinationSideChooser.addOption("Left", "Left");
    secondDestinationSideChooser.addOption("Right", "Right");
    secondDestinationSideChooser.addOption("Center", "Center");

    // Populate third destination type chooser
    thirdDestinationTypeChooser.addDefaultOption("None", "None");
    thirdDestinationTypeChooser.addOption("Depot", "Depot");
    thirdDestinationTypeChooser.addOption("Outpost", "Outpost");
    thirdDestinationTypeChooser.addOption("Fuel Pile", "Fuel Pile");

    // Populate third destination side chooser
    thirdDestinationSideChooser.addDefaultOption("None", "None");
    thirdDestinationSideChooser.addOption("Left", "Left");
    thirdDestinationSideChooser.addOption("Right", "Right");
    thirdDestinationSideChooser.addOption("Center", "Center");

    // This ensures they publish to NetworkTables
    allianceChooser.get();
    startPosChooser.get();
    firstDestinationTypeChooser.get();
    firstDestinationSideChooser.get();
    secondDestinationTypeChooser.get();
    secondDestinationSideChooser.get();
    thirdDestinationTypeChooser.get();
    thirdDestinationSideChooser.get();
  }

  /**
   * Combines destination type and side into a full destination name. Examples: - "Depot" + "Center"
   * = "DC" - "Depot" + "Left" = "DL" - "Fuel Pile Far" + "Center" = "Fuel Pile CF" - "Outpost" +
   * any = "Outpost" (side doesn't matter, can be empty/N/A)
   */
  private String combineDestination(String type, String side) {
    if (type == null || type.equals("None")) {
      return "None";
    }

    if (type.equals("Outpost")) {
      // Outpost doesn't need a side, ignore it even if provided
      return "Outpost";
    }

    if (type.equals("Depot")) {
      if (side == null || side.equals("None")) {
        return "None";
      }
      switch (side) {
        case "Center":
          return "DC";
        case "Left":
          return "DL";
        case "Right":
          return "DR";
      }
    }

    if (type.startsWith("Fuel Pile")) {
      if (side == null || side.equals("None")) {
        return "None";
      }

      // Map side to letter
      String sideLetter = "";
      switch (side) {
        case "Center":
          sideLetter = "C";
          break;
        case "Left":
          sideLetter = "L";
          break;
        case "Right":
          sideLetter = "R";
          break;
      }

      if (!sideLetter.isEmpty()) {
        return "Fuel Pile " + sideLetter;
      }
    }

    return "None";
  }

  public void updateChooserOptions() {

    allianceChooser.get();
    startPosChooser.get();
    firstDestinationTypeChooser.get();
    firstDestinationSideChooser.get();
    secondDestinationTypeChooser.get();
    secondDestinationSideChooser.get();
    thirdDestinationTypeChooser.get();
    thirdDestinationSideChooser.get();
  }

  /**
   * @return The command to run in autonomous, or Commands.none() if no valid selection
   */
  public Command getAutonomousCommand() {
    // Update chooser options first
    updateChooserOptions();

    // Get all selections
    String alliance = allianceChooser.get();
    String startPos = startPosChooser.get();

    String firstDestination =
        combineDestination(firstDestinationTypeChooser.get(), firstDestinationSideChooser.get());
    String secondDestination =
        combineDestination(secondDestinationTypeChooser.get(), secondDestinationSideChooser.get());
    String thirdDestination =
        combineDestination(thirdDestinationTypeChooser.get(), thirdDestinationSideChooser.get());

    // Use DynamicAutoBuilder to chain paths together
    return dynamicAutoBuilder.buildAuto(
        alliance, startPos, firstDestination, secondDestination, thirdDestination);
  }

  /**
   * Returns the starting pose for the currently selected auto by loading the first path and reading
   * its start pose. Use this to set odometry at auto start (e.g. in sim). Empty if no paths or path
   * file missing.
   */
  public Optional<Pose2d> getAutoStartPose() {
    updateChooserOptions();
    String alliance = allianceChooser.get();
    String startPose = startPosChooser.get();
    String firstDestination =
        combineDestination(firstDestinationTypeChooser.get(), firstDestinationSideChooser.get());
    String secondDestination =
        combineDestination(secondDestinationTypeChooser.get(), secondDestinationSideChooser.get());
    String thirdDestination =
        combineDestination(thirdDestinationTypeChooser.get(), thirdDestinationSideChooser.get());

    Optional<String> firstPathName =
        dynamicAutoBuilder.getFirstPathName(
            alliance, startPose, firstDestination, secondDestination, thirdDestination);
    if (firstPathName.isEmpty()) {
      return Optional.empty();
    }
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(firstPathName.get());
      return path.getStartingHolonomicPose();
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  /**
   * Returns the alliance selected in the auto chooser ("B" or "R"). Returns "R" as default if no
   * selection or instance not initialized.
   */
  public static String getAlliance() {
    if (instance == null) {
      return "R";
    }
    String alliance = instance.allianceChooser.get();
    if (alliance == null || alliance.equals("None")) {
      return "R";
    }
    return alliance;
  }
}
