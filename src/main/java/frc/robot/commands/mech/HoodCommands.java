package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.util.Calculations;

public class HoodCommands {

  private HoodCommands() {}

  public static Command RetractHood(HoodSubsystem hoodSubsystem) {
    return new HoodRetractCommand(hoodSubsystem).withName("Retract Hood");
  }

  // The reason this command isn't an instant command is so we don't go under the trench if the hood
  // hasn't finished retracting (that would be very bad)
  public static class HoodRetractCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodRetractCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
      setName("Retract Hood");
    }

    @Override
    public void initialize() {
      hoodSubsystem.setDesiredAngle(HoodConstants.RETRACTED_POSITION);
    }

    @Override
    public void execute() {}

    // is finished if the hood is within the deadband of the target position or if the current limit
    // is reached
    @Override
    public boolean isFinished() {
      if (Calculations.withinDeadband(
          hoodSubsystem.getCurrentAngle().getDegrees(),
          hoodSubsystem.getDesiredAngle().getDegrees(),
          HoodConstants.HOOD_POSITION_DEADBAND_DEGREES,
          true)) {
        return true;
      } else if (hoodSubsystem.isCurrentLimitReached()) {
        hoodSubsystem.setDesiredAngle(hoodSubsystem.getCurrentAngle());
        return true;
      }
      return false;
    }
  }

  /**
   * Stage 1 of hood homing: move fast toward the retracted limit switch and rough-zero when
   * pressed.
   */
  public static class HoodHomingCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodHomingCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
      setName("HoodHomingLimitSwitch");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      hoodSubsystem.setHoodSpeed(HoodConstants.HOMING_SPEED);
    }

    // finish when current limit is reached
    @Override
    public boolean isFinished() {
      return hoodSubsystem.isCurrentLimitReached();
    }

    // zeroes/homes the hood at end of command when isFinished is true or if interrupted
    // maybe if using for reference think about whether you want it to zero when it is interrupted
    @Override
    public void end(boolean interrupted) {
      hoodSubsystem.zeroHood();
      hoodSubsystem.setPositionControl(true);
    }
  }
}
