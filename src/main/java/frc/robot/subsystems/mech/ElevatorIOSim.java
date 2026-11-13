package frc.robot.subsystems.mech;

/** Pretend hardware for unit tests. It remembers what it was told and moves nothing by itself. */
public class ElevatorIOSim implements ElevatorIO {

  /** The last kind of thing the subsystem told the motor to do. */
  public enum Request {
    NONE,
    POSITION,
    VOLTAGE,
    STOP
  }

  private Request lastRequest = Request.NONE;
  private double positionSetpointMeters = 0.0;
  private double voltageVolts = 0.0;
  private int gainUpdates = 0;
  private int positionResets = 0;
  private double lastKp = 0.0;

  // What the pretend hardware reports.
  private boolean connected = true;
  private double positionMeters = 0.0;
  private boolean bottomLimitPressed = false;

  /** Reports whatever the test has set up. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorConnected = connected;
    inputs.positionMeters = positionMeters;
    inputs.bottomLimitPressed = bottomLimitPressed;
  }

  /** Remembers the height. */
  @Override
  public void setPositionSetpoint(double positionMeters) {
    lastRequest = Request.POSITION;
    positionSetpointMeters = positionMeters;
  }

  /** Remembers the voltage. */
  @Override
  public void setVoltage(double volts) {
    lastRequest = Request.VOLTAGE;
    voltageVolts = volts;
  }

  /** Remembers that it was told to stop. */
  @Override
  public void stop() {
    lastRequest = Request.STOP;
  }

  /** Moves the reported height, like the real motor's encoder being reset. */
  @Override
  public void resetPosition(double positionMeters) {
    positionResets++;
    this.positionMeters = positionMeters;
  }

  /** Counts gain updates, and remembers kP so a test can check which gains arrived. */
  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    gainUpdates++;
    lastKp = kP;
  }

  // ---------------- For tests ----------------

  /** The last kind of thing the subsystem told the motor to do. */
  public Request getLastRequest() {
    return lastRequest;
  }

  /** The last height sent with setPositionSetpoint. */
  public double getPositionSetpointMeters() {
    return positionSetpointMeters;
  }

  /** The last voltage sent with setVoltage. */
  public double getVoltageVolts() {
    return voltageVolts;
  }

  /** How many times the position was reset (homing). */
  public int getPositionResets() {
    return positionResets;
  }

  /** How many times new gains were sent. */
  public int getGainUpdates() {
    return gainUpdates;
  }

  /** The kP of the last gains sent. */
  public double getLastKp() {
    return lastKp;
  }

  /** Plugs in or unplugs the pretend motor. */
  public void setConnected(boolean connected) {
    this.connected = connected;
  }

  /** Puts the pretend carriage at a height. */
  public void setPositionMeters(double positionMeters) {
    this.positionMeters = positionMeters;
  }

  /** Presses or releases the pretend bottom limit switch. */
  public void setBottomLimitPressed(boolean pressed) {
    bottomLimitPressed = pressed;
  }
}
