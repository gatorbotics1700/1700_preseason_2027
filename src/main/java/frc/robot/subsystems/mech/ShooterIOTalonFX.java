package frc.robot.subsystems.mech;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.logging.TalonFXLogger;

/** The real hardware: four TalonFXs. All motor configuration lives here. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX flywheelMotor;
  private final TalonFX kickerMotor;
  private final TalonFX leftTransitionMotor;
  private final TalonFX rightTransitionMotor;

  private final TalonFXConfiguration flywheelTalonFXConfigs = new TalonFXConfiguration();
  private final Slot0Configs flywheelSlot0Configs = flywheelTalonFXConfigs.Slot0;

  private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
  private final NeutralOut flywheelNeutralOut = new NeutralOut();
  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0);

  // TODO figure out what these are - claude suggested
  private final StatusSignal<AngularVelocity> flywheelRotorVelocity;
  private final StatusSignal<Angle> flywheelPosition;
  private final StatusSignal<Voltage> flywheelVoltage;
  private final StatusSignal<Current> flywheelCurrent;
  private final StatusSignal<Voltage> kickerVoltage;

  public ShooterIOTalonFX() {
    // directly from subsystem
    flywheelMotor =
        new TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    kickerMotor =
        new TalonFX(ShooterConstants.TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    leftTransitionMotor =
        new TalonFX(ShooterConstants.LEFT_TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    rightTransitionMotor =
        new TalonFX(ShooterConstants.RIGHT_TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    // TALONFX & MOTIONMAGIC CONFIGS    
    flywheelTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));

    flywheelSlot0Configs.kS = 0.37577; // Add __ V output to overcome static friction
    flywheelSlot0Configs.kV = 0.12289; // A velocity target of 1 rps results in 0.12-0.2 V output
    flywheelSlot0Configs.kA = 0.023452;
    flywheelSlot0Configs.kP = ShooterSubsystem.flywheelKP.get();
    flywheelSlot0Configs.kI = ShooterSubsystem.flywheelKI.get();
    flywheelSlot0Configs.kD = ShooterSubsystem.flywheelKD.get();

    // MOTION MAGIC EXPO
    MotionMagicConfigs flywheelMotionMagicConfigs = flywheelTalonFXConfigs.MotionMagic;
    flywheelMotionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    flywheelMotionMagicConfigs.MotionMagicJerk =
        4000; // Target jerk of 4000 rps/s/s (0/1 seconds)

    flywheelTalonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;

    // Kicker (same as the old code: limit value set, enable flag left at its default)
    TalonFXConfiguration transitionMotorConfigs = new TalonFXConfiguration();
    transitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    transitionMotorConfigs.CurrentLimits.StatorCurrentLimit = 80;

    TalonFXConfiguration leftTransitionMotorConfigs = new TalonFXConfiguration();
    leftTransitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    applyTransitionMotorCurrentLimits(leftTransitionMotorConfigs);

    TalonFXConfiguration rightTransitionMotorConfigs = new TalonFXConfiguration();
    rightTransitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    applyTransitionMotorCurrentLimits(rightTransitionMotorConfigs);

    flywheelMotor.getConfigurator().apply(flywheelTalonFXConfigs);
    kickerMotor.getConfigurator().apply(transitionMotorConfigs);
    leftTransitionMotor.getConfigurator().apply(leftTransitionMotorConfigs);
    rightTransitionMotor.getConfigurator().apply(rightTransitionMotorConfigs);

    flywheelRotorVelocity = flywheelMotor.getRotorVelocity();
    flywheelPosition = flywheelMotor.getPosition();
    flywheelVoltage = flywheelMotor.getMotorVoltage();
    flywheelCurrent = flywheelMotor.getStatorCurrent();
    kickerVoltage = kickerMotor.getMotorVoltage();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelConnected =
        BaseStatusSignal.refreshAll(
                flywheelRotorVelocity, flywheelPosition, flywheelVoltage, flywheelCurrent)
            .isOK();
    inputs.kickerConnected = BaseStatusSignal.refreshAll(kickerVoltage).isOK();

    inputs.flywheelRotorRps = flywheelRotorVelocity.getValueAsDouble();
    inputs.flywheelPositionRot = flywheelPosition.getValueAsDouble();
    inputs.flywheelVolts = flywheelVoltage.getValueAsDouble();
    inputs.flywheelAmps = flywheelCurrent.getValueAsDouble();
    inputs.kickerVolts = kickerVoltage.getValueAsDouble();

    // Detailed per-motor logging (same calls as the old shooterLogs())
    TalonFXLogger.log(flywheelMotor, "Mech", "Shooter", "Flywheel");
    TalonFXLogger.log(kickerMotor, "Mech", "Shooter", "Kicker");
    TalonFXLogger.log(leftTransitionMotor, "Mech", "Shooter", "Left Transition");
    TalonFXLogger.log(rightTransitionMotor, "Mech", "Shooter", "Right Transition");
  }

  @Override
  public void setFlywheelVelocity(double rotorRps) {
    flywheelMotor.setControl(velocityRequest.withVelocity(rotorRps));
  }

  @Override
  public void stopFlywheel() {
    flywheelMotor.setControl(flywheelNeutralOut);
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelMotor.setControl(sysIdVoltageRequest.withOutput(volts));
  }

  @Override
  public void setFeederSpeeds(double kicker, double left, double right) {
    kickerMotor.set(kicker);
    leftTransitionMotor.set(left);
    rightTransitionMotor.set(right);
  }

  @Override
  public void setFlywheelGains(double kP, double kI, double kD) {
    flywheelSlot0Configs.kP = kP;
    flywheelSlot0Configs.kI = kI;
    flywheelSlot0Configs.kD = kD;
    flywheelMotor.getConfigurator().apply(flywheelSlot0Configs);
  }

  private static void applyTransitionMotorCurrentLimits(TalonFXConfiguration config) {
    config.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.TRANSITION_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
  }
}