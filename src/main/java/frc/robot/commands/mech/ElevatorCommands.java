package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public final class ElevatorCommands {
  private ElevatorCommands() {}

  /**
   * Drives down slowly until the bottom limit switch closes (the subsystem zeroes the encoder on
   * that edge), then holds at the bottom. Gives up after HOMING_TIMEOUT_SECONDS so a broken switch
   * can't stall the motor forever.
   */
  public static Command home(ElevatorSubsystem elevator) {
    return Commands.run(() -> elevator.setVoltage(ElevatorConstants.HOMING_VOLTS), elevator)
        .until(elevator::isBottomLimitPressed)
        .withTimeout(ElevatorConstants.HOMING_TIMEOUT_SECONDS)
        .andThen(
            Commands.runOnce(
                () -> {
                  if (elevator.isHomed()) {
                    elevator.setGoal(ElevatorConstants.STOW_HEIGHT_METERS);
                  } else {
                    elevator.stop();
                  }
                },
                elevator))
        .withName("ElevatorHome");
  }

  /** Moves to a carriage height and ends once there. The subsystem keeps holding it afterwards. */
  public static Command goTo(ElevatorSubsystem elevator, double heightMeters) {
    return Commands.run(() -> elevator.setGoal(heightMeters), elevator)
        .until(elevator::atGoal)
        .withName("ElevatorGoTo " + heightMeters);
  }

  /** Homes, then visits every setpoint: a quick end-to-end check of the whole mechanism. */
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
