package frc.robot.subsystems.mech;

import org.littletonrobotics.junction.AutoLog;

/**
 * The elevator hardware layer: one motor and a bottom limit switch. Heights are carriage meters
 * above the bottom hard stop.
 *
 * <p>An interface is a list of methods with no code of its own. Each version fills them in:
 *
 * <ul>
 *   <li>ElevatorIOTalonFX: the real motor and switch.
 *   <li>ElevatorIOSimPhysics: the simulator, with physics.
 *   <li>ElevatorIOSim: pretend hardware for unit tests.
 *   <li>new ElevatorIO() {}: does nothing, to turn the elevator off.
 * </ul>
 */
public interface ElevatorIO {

  /** Everything the elevator reads from its hardware. AdvantageKit logs these every loop. */
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motorConnected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean bottomLimitPressed = false;
    // Where Motion Magic's planned path says the carriage should be now.
    public double profileReferenceMeters = 0.0;
  }

  /** Reads the hardware into inputs. Called once at the start of every loop. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Moves to a height with the motor's onboard Motion Magic, and holds it there. */
  public default void setPositionSetpoint(double positionMeters) {}

  /** Drives the motor at a voltage. Negative is down. */
  public default void setVoltage(double volts) {}

  /** Stops driving the motor. Brake mode holds the carriage still. */
  public default void stop() {}

  /** Tells the motor the carriage is at this height. Used when homing. */
  public default void resetPosition(double positionMeters) {}

  /** Sends new closed loop gains to the motor. */
  public default void setGains(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {}
}
