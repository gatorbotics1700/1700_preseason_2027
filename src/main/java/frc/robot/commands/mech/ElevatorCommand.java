package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.mech.ElevatorSubsystem;

// TODO: Add buttons in RobotContainer to call these commands when the elevator is moved up or down
public class ElevatorCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;

  // Allowed position error in motor rotations
  private static final double ERROR = 1.0;

  public ElevatorCommand() {
    this.elevatorSubsystem = new ElevatorSubsystem();
  }

  public Command HomeElevatorCommand(){
    return new InstantCommand(
        () -> {
          elevatorSubsystem.HomeElevator(Constants.ElevatorConstants.ELEVATOR_HOMING_SPEED);
        },
        elevatorSubsystem
    ).until(() -> elevatorSubsystem.isAtBottomLimit());

  }
  public Command StopElevatorCommand(){
    return new InstantCommand(
        () -> {
          elevatorSubsystem.stop();
        },
        elevatorSubsystem
    );
  }

  public Command MoveElevatorCommand(double desiredHeight){
    return new InstantCommand(
        () -> {
          elevatorSubsystem.moveToPosition(elevatorSubsystem.getElevatorPosition(), desiredHeight);
        },
        elevatorSubsystem
    ).until(() -> elevatorSubsystem.isAtTopLimit()|| elevatorSubsystem.isAtBottomLimit());
  }

}
