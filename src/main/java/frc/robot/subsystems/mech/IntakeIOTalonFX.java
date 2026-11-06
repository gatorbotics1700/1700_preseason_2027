package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.logging.TalonFXLogger;

/**
 * The real intake hardware: four TalonFX motors and two limit switches. This class only passes
 * values to and from the hardware; all decisions stay in IntakeSubsystem. The code here was moved
 * out of IntakeSubsystem unchanged.
 */
public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;

  private final TalonFXConfiguration leftRollerConfigs;
  private final TalonFXConfiguration rightRollerConfigs;
  private final CurrentLimitsConfigs leftRollerCurrentLimitConfigs;
  private final CurrentLimitsConfigs rightRollerCurrentLimitConfigs;

  // 0 means no top-speed cap: the Expo gains alone shape the move.
  private static final double DEPLOY_CRUISE_VELOCITY_RPS = 0;

  private final TalonFX leftDeployMotor =
      new TalonFX(IntakeConstants.INTAKE_LEFT_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  private final TalonFX rightDeployMotor =
      new TalonFX(IntakeConstants.INTAKE_RIGHT_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);

  // Both deploy motors share one settings sheet. The three below are sections of that sheet.
  private final TalonFXConfiguration deployConfigs = new TalonFXConfiguration();
  private final Slot0Configs deployGains = deployConfigs.Slot0;
  private final MotionMagicConfigs deployMotionMagic = deployConfigs.MotionMagic;
  private final CurrentLimitsConfigs deployCurrentLimit = deployConfigs.CurrentLimits;

  // One reusable "go to this position" request; the target is filled in each time it is sent.
  private final MotionMagicExpoVoltage deployRequest = new MotionMagicExpoVoltage(0);

  // NOTE (2026-11-02), CHECK WITH BUILD TEAM: get() == true is treated as "pressed". roboRIO
  // ports read true when nothing is connected, so check which way these sensors report.
  private final DigitalInput retractedLimitSwitch =
      new DigitalInput(IntakeConstants.RETRACTED_LIMIT_SWITCH_PORT);
  private final DigitalInput deployedLimitSwitch =
      new DigitalInput(IntakeConstants.DEPLOYED_LIMIT_SWITCH_PORT);

  /** Sets up the rollers and the deploy arm motors. */
  public IntakeIOTalonFX() {
    leftRollerMotor =
        new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    rightRollerMotor =
        new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    // The two rollers face each other, so one spins the opposite way to pull in the same direction.
    leftRollerConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    rightRollerConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    leftRollerCurrentLimitConfigs = leftRollerConfigs.CurrentLimits;
    leftRollerCurrentLimitConfigs.StatorCurrentLimit =
        IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;
    leftRollerCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    rightRollerCurrentLimitConfigs = rightRollerConfigs.CurrentLimits;
    rightRollerCurrentLimitConfigs.StatorCurrentLimit =
        IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;
    rightRollerCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    leftRollerMotor.getConfigurator().apply(leftRollerConfigs);
    rightRollerMotor.getConfigurator().apply(rightRollerConfigs);

    leftRollerMotor.set(0);
    rightRollerMotor.set(0);

    setUpDeployMotors();
  }

  /**
   * Sets the deploy motors' direction, current limit and top speed. The gains start at 0;
   * IntakeSubsystem sends the dashboard gains right after it is built.
   */
  private void setUpDeployMotors() {
    deployConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    deployMotionMagic.MotionMagicCruiseVelocity = DEPLOY_CRUISE_VELOCITY_RPS;
    deployCurrentLimit.StatorCurrentLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT_AMPS;
    deployCurrentLimit.StatorCurrentLimitEnable = true;
    applyDeployConfigs();
  }

  /**
   * Sends the deploy settings sheet to both deploy motors. This is slow, so only do it on change.
   */
  private void applyDeployConfigs() {
    leftDeployMotor.getConfigurator().apply(deployConfigs);
    rightDeployMotor.getConfigurator().apply(deployConfigs);
  }

  /** Reads the rollers, the deploy arm and the limit switches. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    readRollers(inputs);
    readDeployArm(inputs);
    inputs.retractedSwitchPressed = retractedLimitSwitch.get();
    inputs.deployedSwitchPressed = deployedLimitSwitch.get();
  }

  /** Reads the roller motors. */
  private void readRollers(IntakeIOInputs inputs) {
    inputs.leftRollerConnected = leftRollerMotor.isConnected();
    inputs.rightRollerConnected = rightRollerMotor.isConnected();
    inputs.leftRollerCurrentAmps = leftRollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightRollerCurrentAmps = rightRollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftRollerCurrentLimitAmps = leftRollerCurrentLimitConfigs.StatorCurrentLimit;

    // Kept from the old code so the existing "Left Intake" / "Right Intake" logs do not change.
    TalonFXLogger.log(leftRollerMotor, "Mech", "Intake", "Left Intake");
    TalonFXLogger.log(rightRollerMotor, "Mech", "Intake", "Right Intake");
  }

  /** Reads the deploy motors. */
  private void readDeployArm(IntakeIOInputs inputs) {
    inputs.leftDeployConnected = leftDeployMotor.isConnected();
    inputs.rightDeployConnected = rightDeployMotor.isConnected();
    inputs.leftDeployPositionRotations = leftDeployMotor.getPosition().getValueAsDouble();
    inputs.leftDeployVelocityRps = leftDeployMotor.getVelocity().getValueAsDouble();
    inputs.leftDeployVoltageVolts = leftDeployMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftDeployCurrentAmps = leftDeployMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightDeployCurrentAmps = rightDeployMotor.getStatorCurrent().getValueAsDouble();
    inputs.deployCurrentLimitAmps = deployCurrentLimit.StatorCurrentLimit;

    // Kept from the old code so the existing "Left Deploy" / "Right Deploy" logs do not change.
    TalonFXLogger.log(leftDeployMotor, "Mech", "Intake", "Left Deploy");
    TalonFXLogger.log(rightDeployMotor, "Mech", "Intake", "Right Deploy");
  }

  /** Runs both rollers at the same speed. */
  @Override
  public void setRollerSpeed(double speed) {
    leftRollerMotor.set(speed);
    rightRollerMotor.set(speed);
  }

  /**
   * Applies a new roller current limit, only when it changed.
   *
   * <p>KNOWN QUIRK, kept on purpose so this move changes nothing: only the RIGHT roller gets the
   * new limit. No test covers this, because tests use IntakeIOSim; check it on the robot.
   */
  @Override
  public void setRollerCurrentLimitAmps(double amps) {
    if (amps != rightRollerCurrentLimitConfigs.StatorCurrentLimit) {
      rightRollerCurrentLimitConfigs.StatorCurrentLimit = amps;
      rightRollerMotor.getConfigurator().apply(rightRollerConfigs);
    }
  }

  /** Runs both deploy motors at the same speed. */
  @Override
  public void setDeploySpeed(double speed) {
    leftDeployMotor.set(speed);
    rightDeployMotor.set(speed);
  }

  /** Sends both deploy motors to the same position. Each motor plans its own smooth move. */
  @Override
  public void setDeployTargetRotations(double rotations) {
    leftDeployMotor.setControl(deployRequest.withPosition(rotations));
    rightDeployMotor.setControl(deployRequest.withPosition(rotations));
  }

  /** Runs both deploy motors at the same voltage. */
  @Override
  public void setDeployVoltage(double volts) {
    leftDeployMotor.setVoltage(volts);
    rightDeployMotor.setVoltage(volts);
  }

  /** Tells both deploy motors they are now at this position. */
  @Override
  public void resetDeployPositionRotations(double rotations) {
    leftDeployMotor.setPosition(rotations);
    rightDeployMotor.setPosition(rotations);
  }

  /** Sends new PID gains to the deploy motors, only when they changed. */
  @Override
  public void setDeployPidGains(double kP, double kI, double kD) {
    if (kP != deployGains.kP || kI != deployGains.kI || kD != deployGains.kD) {
      deployGains.kP = kP;
      deployGains.kI = kI;
      deployGains.kD = kD;
      applyDeployConfigs();
    }
  }

  /** Sends new feedforward gains to the deploy motors, only when they changed. */
  @Override
  public void setDeployFeedforwardGains(double kG, double kS, double kV, double kA) {
    if (kG != deployGains.kG
        || kS != deployGains.kS
        || kV != deployGains.kV
        || kA != deployGains.kA) {
      deployGains.kG = kG;
      deployGains.kS = kS;
      deployGains.kV = kV;
      deployGains.kA = kA;
      applyDeployConfigs();
    }
  }

  /** Sends new Motion Magic Expo gains to the deploy motors, only when they changed. */
  @Override
  public void setDeployMotionMagicExpoGains(double kV, double kA) {
    if (kV != deployMotionMagic.MotionMagicExpo_kV || kA != deployMotionMagic.MotionMagicExpo_kA) {
      deployMotionMagic.MotionMagicExpo_kV = kV;
      deployMotionMagic.MotionMagicExpo_kA = kA;
      applyDeployConfigs();
    }
  }
}
