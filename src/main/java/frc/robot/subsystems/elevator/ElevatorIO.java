package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the elevator. All positions are carriage height in meters above the
 * bottom hard stop. Implementations: {@link ElevatorIOTalonFX} (real robot), {@link ElevatorIOSim}
 * (physics sim), and the empty default (log replay, or a robot without an elevator).
 */
public interface ElevatorIO {
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

    /** Motion Magic's current profile point (what the controller is tracking right now). */
    public double profileReferenceMeters = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Runs onboard Motion Magic to the carriage height. */
  public default void setPositionSetpoint(double positionMeters) {}

  /** Runs the motor open loop at the given voltage. */
  public default void setVoltage(double volts) {}

  /** Stops driving the motor (neutral output, brake mode holds it). */
  public default void stop() {}

  /** Tells the encoder the carriage is at this height (used when homing). */
  public default void resetPosition(double positionMeters) {}

  /** Applies new closed loop gains. */
  public default void setGains(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {}
}
