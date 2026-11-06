package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TunerConstants;

/**
 * Real elevator: one TalonFX running Motion Magic onboard, plus a bottom limit switch on the
 * roboRIO. The TalonFX's SensorToMechanismRatio is set so its "rotations" are meters of carriage
 * travel, which keeps every gain and setpoint in meters.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  protected final TalonFX motor =
      new TalonFX(ElevatorConstants.MOTOR_CAN_ID, TunerConstants.mechCANBus);
  protected final DigitalInput bottomLimit =
      new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_DIO);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
  private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();
  private final StatusSignal<Double> profileReference = motor.getClosedLoopReference();

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // + is up
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.ROTOR_ROTATIONS_PER_METER;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = ElevatorConstants.KP;
    config.Slot0.kI = ElevatorConstants.KI;
    config.Slot0.kD = ElevatorConstants.KD;
    config.Slot0.kS = ElevatorConstants.KS;
    config.Slot0.kG = ElevatorConstants.KG;
    config.Slot0.kV = ElevatorConstants.KV;
    config.Slot0.kA = ElevatorConstants.KA;

    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY_MPS;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION_MPS2;
    config.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK_MPS3;

    // Soft limits back up the clamp in ElevatorSubsystem. They only make sense once homed, which
    // is why the reverse limit is just below zero rather than exactly at it.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT_METERS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, statorCurrent, supplyCurrent, profileReference);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, temperature);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            appliedVolts,
            statorCurrent,
            supplyCurrent,
            temperature,
            profileReference);
    inputs.motorConnected = connectedDebounce.calculate(status.isOK());
    inputs.positionMeters = position.getValueAsDouble();
    inputs.velocityMetersPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temperature.getValueAsDouble();
    inputs.profileReferenceMeters = profileReference.getValueAsDouble();
    inputs.bottomLimitPressed = bottomLimit.get() == ElevatorConstants.LIMIT_SWITCH_PRESSED_VALUE;
  }

  @Override
  public void setPositionSetpoint(double positionMeters) {
    motor.setControl(positionRequest.withPosition(positionMeters));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void resetPosition(double positionMeters) {
    motor.setPosition(positionMeters);
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kG = kG;
    slot0.kV = kV;
    slot0.kA = kA;
    motor.getConfigurator().apply(slot0, 0.0);
  }
}
