package frc.robot.subsystems.mech;

import org.littletonrobotics.junction.AutoLog;

/**
 * The intake's hardware layer. IntakeSubsystem makes the decisions and talks to the hardware only
 * through this interface, so a simulated version can stand in for real motors.
 *
 * <p>An interface is a list of methods with no hardware code of its own. Each version fills them
 * in:
 *
 * <ul>
 *   <li>IntakeIOTalonFX: the real motors. The simulator also uses it, on CTRE's simulated motors.
 *   <li>IntakeIOSim: pretend hardware that remembers what it was told. Used only by tests.
 *   <li>new IntakeIO() {}: does nothing at all, because every method below has an empty default.
 *       Use it to turn the intake off without unplugging anything.
 * </ul>
 *
 * <p>Everything the intake owns goes through here: the two rollers, the two deploy arm motors, and
 * the two limit switches.
 */
public interface IntakeIO {

  /** Everything the intake reads from its hardware. AdvantageKit logs these every loop. */
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leftRollerConnected = false;
    public boolean rightRollerConnected = false;
    // No code reads the two roller currents. They are logged to look at, for example after a jam.
    public double leftRollerCurrentAmps = 0.0;
    public double rightRollerCurrentAmps = 0.0;
    // The LEFT roller's limit. On the real robot the dashboard limit only changes the RIGHT roller
    // (see IntakeIOTalonFX.setRollerCurrentLimitAmps), so this stays at the starting limit.
    public double leftRollerCurrentLimitAmps = 0.0;

    public boolean leftDeployConnected = false;
    public boolean rightDeployConnected = false;
    // The arm's angle is worked out from the LEFT deploy motor only.
    public double leftDeployPositionRotations = 0.0;
    public double leftDeployVelocityRps = 0.0;
    public double leftDeployVoltageVolts = 0.0;
    public double leftDeployCurrentAmps = 0.0;
    public double rightDeployCurrentAmps = 0.0;
    public double deployCurrentLimitAmps = 0.0;

    public boolean retractedSwitchPressed = false;
    public boolean deployedSwitchPressed = false;
  }

  /** Reads the hardware and fills in the inputs. Called once every loop. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Runs both rollers at a speed from -1 (full reverse) to 1 (full forward). */
  public default void setRollerSpeed(double speed) {}

  /** Sets the rollers' stator current limit, in amps. */
  public default void setRollerCurrentLimitAmps(double amps) {}

  /** Runs both deploy motors at a speed from -1 (retract) to 1 (deploy). */
  public default void setDeploySpeed(double speed) {}

  /** Sends both deploy motors to a position, in motor rotations, moving smoothly. */
  public default void setDeployTargetRotations(double rotations) {}

  /** Runs both deploy motors at a voltage. Only SysId tuning uses this. */
  public default void setDeployVoltage(double volts) {}

  /** Tells both deploy motors where they are now, in motor rotations. Used when homing. */
  public default void resetDeployPositionRotations(double rotations) {}

  /** Sets the deploy arm's PID gains. */
  public default void setDeployPidGains(double kP, double kI, double kD) {}

  /** Sets the deploy arm's feedforward gains. */
  public default void setDeployFeedforwardGains(double kG, double kS, double kV, double kA) {}

  /** Sets how gently the deploy arm speeds up and slows down on its way to a position. */
  public default void setDeployMotionMagicExpoGains(double kV, double kA) {}
}
