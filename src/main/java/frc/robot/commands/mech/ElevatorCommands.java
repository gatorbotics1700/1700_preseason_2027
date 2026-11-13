package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.mech.ElevatorSubsystem;

/** The elevator's actions. Each one is a command a button or an auto can run. */
public final class ElevatorCommands {
  private ElevatorCommands() {}

  /** Homes the elevator on the bottom limit switch and ends when homing is done or gives up. */
  public static Command home(ElevatorSubsystem elevator) {
    Command start = Commands.runOnce(elevator::startHoming, elevator);
    Command waitUntilDone = Commands.waitUntil(() -> !elevator.isHoming());
    return start.andThen(waitUntilDone).withName("ElevatorHome");
  }

  /** Moves to a carriage height and ends once there. The subsystem keeps holding it afterwards. */
  public static Command goTo(ElevatorSubsystem elevator, double heightMeters) {
    return Commands.run(() -> elevator.setGoal(heightMeters), elevator)
        .until(elevator::atGoal)
        .withName("ElevatorGoTo " + heightMeters);
  }

  /** Homes, then visits every setpoint, as a quick check of the whole mechanism. */
  public static Command demoCycle(ElevatorSubsystem elevator) {
    double dwell = 0.5;
    return Commands.sequence(
            home(elevator),
            goTo(elevator, ElevatorConstants.LOW_HEIGHT_METERS),
            Commands.waitSeconds(dwell),
            goTo(elevator, ElevatorConstants.HIGH_HEIGHT_METERS),
            Commands.waitSeconds(dwell),
            goTo(elevator, ElevatorConstants.MID_HEIGHT_METERS),
            Commands.waitSeconds(dwell),
            goTo(elevator, ElevatorConstants.STOW_HEIGHT_METERS),
            Commands.waitSeconds(dwell))
        .withName("ElevatorDemoCycle");
  }
}
