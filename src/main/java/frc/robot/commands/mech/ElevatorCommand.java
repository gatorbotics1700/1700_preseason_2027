package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ElevatorSubsystem;

// TODO set my meters instead of rotations
public class ElevatorCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final double rotations;

  // Allowed position error in motor rotations
  private static final double ERROR = 1.0;

  public ElevatorCommand(double rotations, ElevatorSubsystem elevatorSubsystem) {
    this.rotations = rotations;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setDesiredPosition(rotations); // use move to position
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getIsHomed() && elevatorSubsystem.atDesiredPosition(rotations, ERROR);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      elevatorSubsystem.stop();
    }
  }
}
