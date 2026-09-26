package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ElevatorSubsystem;

public class ElevatorCommand extends Command {

  private int rotations;
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, int rotations) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
    this.rotations = rotations;
  }

  public void initialize() {
    elevatorSubsystem.setTargetPosition(rotations);
  }

  public void execute() {}

  public boolean isFinished() {
    return elevatorSubsystem.hasReachedTarget(rotations);
  }

  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }
}
