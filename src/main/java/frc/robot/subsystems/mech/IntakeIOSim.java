package frc.robot.subsystems.mech;

import frc.robot.Constants.IntakeConstants;

/**
 * Pretend intake hardware for tests and simulation. It has no motors: it just remembers what the
 * intake told it, so a test can check it right away, with no waiting.
 *
 * <p>The arm does not move by itself here. A test moves it with {@link #moveArmToRotations} and
 * presses the switches with {@link #setRetractedSwitchPressed} and {@link
 * #setDeployedSwitchPressed}. Both switches start released.
 */
public class IntakeIOSim implements IntakeIO {

  private double rollerSpeed = 0.0;
  // Starts at the same limit as the real rollers.
  private double rollerCurrentLimitAmps = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;

  private double deploySpeed = 0.0;
  private double deployTargetRotations = 0.0;
  private boolean deployDrivenToTarget = false;
  private double deployVoltageVolts = 0.0;
  private double deployPositionRotations = 0.0;
  private double deployCurrentAmps = 0.0;

  private boolean retractedSwitchPressed = false;
  private boolean deployedSwitchPressed = false;

  // Every motor starts connected. A test can unplug one to check the alerts.
  private boolean leftRollerConnected = true;
  private boolean rightRollerConnected = true;
  private boolean leftDeployConnected = true;
  private boolean rightDeployConnected = true;

  /** Reports whatever the test has set up. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftRollerConnected = leftRollerConnected;
    inputs.rightRollerConnected = rightRollerConnected;
    inputs.leftRollerCurrentAmps = 0.0;
    inputs.rightRollerCurrentAmps = 0.0;
    inputs.leftRollerCurrentLimitAmps = rollerCurrentLimitAmps;

    inputs.leftDeployConnected = leftDeployConnected;
    inputs.rightDeployConnected = rightDeployConnected;
    inputs.leftDeployPositionRotations = deployPositionRotations;
    inputs.leftDeployVelocityRps = 0.0;
    inputs.leftDeployVoltageVolts = deployVoltageVolts;
    inputs.leftDeployCurrentAmps = deployCurrentAmps;
    inputs.rightDeployCurrentAmps = deployCurrentAmps;
    inputs.deployCurrentLimitAmps = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT_AMPS;

    inputs.retractedSwitchPressed = retractedSwitchPressed;
    inputs.deployedSwitchPressed = deployedSwitchPressed;
  }

  /** Remembers the roller speed. */
  @Override
  public void setRollerSpeed(double speed) {
    rollerSpeed = speed;
  }

  /** Remembers the roller current limit. */
  @Override
  public void setRollerCurrentLimitAmps(double amps) {
    rollerCurrentLimitAmps = amps;
  }

  /** Remembers the arm speed. */
  @Override
  public void setDeploySpeed(double speed) {
    deploySpeed = speed;
    deployDrivenToTarget = false;
  }

  /** Remembers the arm's target position. */
  @Override
  public void setDeployTargetRotations(double rotations) {
    deployTargetRotations = rotations;
    deployDrivenToTarget = true;
  }

  /** Remembers the arm voltage. */
  @Override
  public void setDeployVoltage(double volts) {
    deployVoltageVolts = volts;
    deployDrivenToTarget = false;
  }

  /** The arm's position reading is reset, as when homing on a switch. */
  @Override
  public void resetDeployPositionRotations(double rotations) {
    deployPositionRotations = rotations;
  }

  // ---------------- For tests ----------------

  /** The last roller speed the intake asked for. */
  public double getRollerSpeed() {
    return rollerSpeed;
  }

  /** The last roller current limit the intake asked for, in amps. */
  public double getRollerCurrentLimitAmps() {
    return rollerCurrentLimitAmps;
  }

  /** The last arm speed the intake asked for. */
  public double getDeploySpeed() {
    return deploySpeed;
  }

  /** The last arm target the intake asked for, in motor rotations. */
  public double getDeployTargetRotations() {
    return deployTargetRotations;
  }

  /** True if the arm was last told to go to a position, false if told a speed or voltage. */
  public boolean isDeployDrivenToTarget() {
    return deployDrivenToTarget;
  }

  /** Moves the pretend arm, as if it had swung there. */
  public void moveArmToRotations(double rotations) {
    deployPositionRotations = rotations;
  }

  /** Sets the current both arm motors report, in amps. Use a high value to fake a jam. */
  public void setDeployCurrentAmps(double amps) {
    deployCurrentAmps = amps;
  }

  /** Presses or releases the retracted switch. */
  public void setRetractedSwitchPressed(boolean pressed) {
    retractedSwitchPressed = pressed;
  }

  /** Presses or releases the deployed switch. */
  public void setDeployedSwitchPressed(boolean pressed) {
    deployedSwitchPressed = pressed;
  }

  /** Plugs in or unplugs the pretend left roller motor. */
  public void setLeftRollerConnected(boolean connected) {
    leftRollerConnected = connected;
  }

  /** Plugs in or unplugs the pretend right roller motor. */
  public void setRightRollerConnected(boolean connected) {
    rightRollerConnected = connected;
  }

  /** Plugs in or unplugs the pretend left deploy motor. */
  public void setLeftDeployConnected(boolean connected) {
    leftDeployConnected = connected;
  }

  /** Plugs in or unplugs the pretend right deploy motor. */
  public void setRightDeployConnected(boolean connected) {
    rightDeployConnected = connected;
  }
}
