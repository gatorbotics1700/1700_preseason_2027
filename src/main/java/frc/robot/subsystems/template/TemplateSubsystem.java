package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * TEMPLATE: the decisions for a mechanism with one motor that runs at a set speed. It talks to the
 * hardware only through TemplateIO, so tests and the simulator can use TemplateIOSim.
 *
 * <p>It is not built in RobotContainer, so it never runs on the robot. Copy this folder to start a
 * new mechanism; see README.md.
 */
public class TemplateSubsystem extends SubsystemBase {

  // The hardware layer: real, simulated, or one that does nothing.
  private final TemplateIO io;
  private final TemplateIOInputsAutoLogged inputs = new TemplateIOInputsAutoLogged();

  // Dashboard warning. Not private, so the tests in this folder can check it.
  // TODO: put your mechanism's name and CAN ID in the message, like the intake's alerts.
  final Alert motorDisconnectedAlert = new Alert("Template: motor disconnected.", AlertType.kError);

  private double desiredSpeed = 0.0;

  /** Builds the mechanism with the real hardware. RobotContainer would use this one. */
  public TemplateSubsystem() {
    this(new TemplateIOTalonFX());
  }

  /** Builds the mechanism with any hardware layer: real, simulated, or one that does nothing. */
  public TemplateSubsystem(TemplateIO io) {
    this.io = io;
  }

  /** Runs every 20 ms: read the hardware, log it, then check for problems. */
  @Override
  public void periodic() {
    // Read the hardware first, so every decision below uses this loop's values.
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Template/IO", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);

    // TODO: your mechanism's decisions go here. Use only inputs, never the motor directly.

    // What it was told to do. What it did is already logged in inputs above.
    Logger.recordOutput("Mech/Template/Desired Speed", desiredSpeed);
  }

  /** Runs the motor at a speed from -1 (full reverse) to 1 (full forward). */
  public void setSpeed(double speed) {
    desiredSpeed = speed;
    io.setSpeed(speed);
  }

  /** Stops the motor. */
  public void stop() {
    setSpeed(0);
  }
}
