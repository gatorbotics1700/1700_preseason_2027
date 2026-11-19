package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.mech.ElevatorSubsystem;

// TODO: Add buttons in RobotContainer to call these commands when the elevator is moved up or down
public class ElevatorCommand extends Command {

  // Allowed position error in motor rotations
  private static final double ERROR = 1.0;

  public ElevatorCommand() {}

  public Command HomeElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    return new InstantCommand(
            () -> {
              elevatorSubsystem.homeElevator(Constants.ElevatorConstants.ELEVATOR_HOMING_SPEED);
            },
            elevatorSubsystem)
        .until(() -> elevatorSubsystem.isAtBottomLimit())
        .andThen(() -> elevatorSubsystem.zeroElevator());
  }

  public Command StopElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    return new InstantCommand(
        () -> {
          elevatorSubsystem.stop();
        },
        elevatorSubsystem);
  }

  public Command MoveElevatorCommand(double desiredHeight, ElevatorSubsystem elevatorSubsystem) {
    return new InstantCommand(
            () -> {
              elevatorSubsystem.moveToPosition(
                  elevatorSubsystem.getElevatorPosition(), desiredHeight);
            },
            elevatorSubsystem)
        .until(() -> elevatorSubsystem.isAtTopLimit() || elevatorSubsystem.isAtBottomLimit());
  }
}
