package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.WristSubsystem;

// TODO imports

public class WristCommands {

  public WristCommands() {}

  // we need a command to stop wrist, pivot the wrist, and rotate the rist
  public static class StopWristCommand extends Command {

    private final WristSubsystem wristSubsystem;
    private boolean executed;

    public StopWristCommand(WristSubsystem wristSubsystem) {
      this.wristSubsystem = wristSubsystem;
      addRequirements(wristSubsystem);
      executed = false;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      wristSubsystem.setVoltage(0.0, 0.0);
      executed = true;
    }

    @Override
    public boolean isFinished() {
      return (wristSubsystem.getRightWristVoltage()==0.0 && wristSubsystem.getLeftWristVoltage()==0.0); //checks if voltage is actually at 0
      //TODO: make sure this works if not just use executed boolean
    }
  }

  public static class PivotWristCommand extends Command {

    private final WristSubsystem wristSubsystem;
    private boolean executed;
    private final Rotation2d desiredPosition;

    public PivotWristCommand(WristSubsystem wristSubsystem, Rotation2d desiredPosition) {
      this.wristSubsystem = wristSubsystem;
      addRequirements(wristSubsystem);
      this.desiredPosition = desiredPosition;
      executed = false;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      wristSubsystem.setPivot(desiredPosition);
      executed = true;
    }

    @Override
    public boolean isFinished() {
      return executed; //this should check if the wrist is actually in the desired position
    }
  }

  public static class RotateWristCommand extends Command {

    private final WristSubsystem wristSubsystem;
    private boolean executed;
    private final Rotation2d desiredPosition;

    public RotateWristCommand(WristSubsystem wristSubsystem, Rotation2d desiredPosition) {
      this.wristSubsystem = wristSubsystem;
      addRequirements(wristSubsystem);
      this.desiredPosition = desiredPosition;
      executed = false;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      wristSubsystem.setRotation(desiredPosition);
      executed = true;
    }

    @Override
    public boolean isFinished() {
      return executed; //this should check if the wrist is actually in the desired position
    }
  }

}
