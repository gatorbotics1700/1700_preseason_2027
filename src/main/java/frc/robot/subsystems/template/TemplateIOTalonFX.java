package frc.robot.subsystems.template;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.TunerConstants;

/**
 * TEMPLATE: the real hardware, one TalonFX motor. This class only passes values to and from the
 * motor; all decisions stay in TemplateSubsystem.
 */
public class TemplateIOTalonFX implements TemplateIO {

  // TODO: move to the config file when you copy this (see README.md). This class is never built on
  // the robot, so this placeholder never reaches a real motor.
  private static final int MOTOR_CAN_ID = 0;

  // TODO: move to Constants.java when you copy this (see README.md).
  // PLACEHOLDERS, TUNED, NOT DERIVED (2026-11-06): ask a mentor for your mechanism's limits.
  // Stator limit: caps the motor's own current, which limits torque and heat.
  private static final double STATOR_CURRENT_LIMIT_AMPS = 40;
  // Supply limit: caps what the motor draws from the battery, to help prevent brownouts.
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 30;

  // How many times to try sending the settings. They can fail right after power-on.
  private static final int CONFIG_ATTEMPTS = 5;
  private static final double CONFIG_TIMEOUT_SECONDS = 0.25;

  private final TalonFX motor = new TalonFX(MOTOR_CAN_ID, TunerConstants.mechCANBus);

  /** Sets up the motor once. */
  public TemplateIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Rollers usually coast. Arms, turrets and hoods usually brake. Choose on purpose.
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TODO: check which way is "forward" on your mechanism.
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Settings are slow to send, so send them once, here, and retry if the motor says no.
    // KNOWN GAP: if every try fails, tryUntilOk gives up quietly, with no alert (see README.md).
    tryUntilOk(
        CONFIG_ATTEMPTS, () -> motor.getConfigurator().apply(config, CONFIG_TIMEOUT_SECONDS));
  }

  /** Reads the motor. */
  @Override
  public void updateInputs(TemplateIOInputs inputs) {
    inputs.motorConnected = motor.isConnected();
    inputs.motorVelocityRps = motor.getVelocity().getValueAsDouble();
    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  /** Runs the motor at a speed from -1 to 1. */
  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
