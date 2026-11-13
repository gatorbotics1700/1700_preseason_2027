package frc.robot.subsystems.mech;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO: make the elevator constant clearer by detailing the multiplications that happen
public class ElevatorSubsystem extends SubsystemBase {
  
  private final TalonFX elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID);

  private final DigitalInput topLimitSwitch = new DigitalInput(8);

  private final DigitalInput bottomLimitSwitch = new DigitalInput(9);

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

  public static final LoggedNetworkNumber elevatorKp =
      new LoggedNetworkNumber("/Tuning/Elevator/kP", 30.0);

  public static final LoggedNetworkNumber elevatorKi =
      new LoggedNetworkNumber("/Tuning/Elevator/kI", 0.0);

  public static final LoggedNetworkNumber elevatorKd =
      new LoggedNetworkNumber("/Tuning/Elevator/kD", 0.1);

  public static final LoggedNetworkNumber elevatorKg =
      new LoggedNetworkNumber("/Tuning/Elevator/kG", 0.2);

  public static final LoggedNetworkNumber elevatorKs =
      new LoggedNetworkNumber("/Tuning/Elevator/kS", 0.01);

  public static final LoggedNetworkNumber elevatorKv =
      new LoggedNetworkNumber("/Tuning/Elevator/kV", 0.16);

  public static final LoggedNetworkNumber elevatorKa =
      new LoggedNetworkNumber("/Tuning/Elevator/kA", 0.01);

  private double targetPosition = 0.0;
  private boolean isHomed = false;
  private boolean wasAtBottom = false;

  public ElevatorSubsystem() {

    motorConfig.MotorOutput.Inverted = Constants.ElevatorConstants.MOTOR_DIRECTION;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0 = motorConfig.Slot0;

    slot0.GravityType = GravityTypeValue.Elevator_Static;

    slot0.kP = elevatorKp.get();
    slot0.kI = elevatorKi.get();
    slot0.kD = elevatorKd.get();

    slot0.kG = elevatorKg.get();
    slot0.kS = elevatorKs.get();
    slot0.kV = elevatorKv.get();
    slot0.kA = elevatorKa.get();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    motorConfig.MotionMagic.MotionMagicExpo_kV = 0.16;
    motorConfig.MotionMagic.MotionMagicExpo_kA = 0.2;

    //TODO: figure out soft limit threshold logic
    //motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = metersToMotorPosition(Constants.ElevatorConstants.MIN_HEIGHT);

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    elevatorMotor.getConfigurator().apply(motorConfig);

    if (isAtBottomLimit()) {
      zeroElevator();
    }
  }

  @Override
  public void periodic() {

    updatePID();

    boolean atBottom = isAtBottomLimit();
    boolean atTop = isAtTopLimit();

    if (atBottom && !wasAtBottom) {
      zeroElevator();
    }

    double motorVelocity = elevatorMotor.getVelocity().getValueAsDouble();

    if (atTop && motorVelocity > 0) {
      stop();
    }

    if (atBottom && motorVelocity < 0) {
      stop();
    }
  }

  private void logElevatorMethods() {
    SmartDashboard.putBoolean("Elevator/AtTarget", isAtTargetPosition());

    SmartDashboard.putNumber("Elevator/HeightMeters", getElevatorPosition());

    SmartDashboard.putNumber("Elevator/TargetMeters", targetPosition);

    SmartDashboard.putNumber("Elevator/MotorRotations", getCurrentPosition());

    SmartDashboard.putBoolean("Elevator/Homed", isHomed);

    Logger.recordOutput("Commands/topLimitSwitch", isAtBottomLimit());

    Logger.recordOutput("Commands/bottomLimitSwitch", isAtTopLimit());
  }


  private double heightToRevs(double height) { // height is in inches, returns revs of the motor
    return height / Constants.ElevatorConstants.HEIGHT_TO_REVS;
  }

  private double revsToHeight(double revs) { // revs of the motor, returns height in inches
    return revs * Constants.ElevatorConstants.HEIGHT_TO_REVS;
  }

  private void goToHeight(double currentHeight, double desiredHeight) { // height is in inches
    double revs = heightToRevs(desiredHeight - currentHeight);
    elevatorMotor.setControl(motionMagicRequest.withPosition(revs));
  }

  public void moveToPosition(double currentHeight, double desiredHeight) { // height is in inches
    if (!isHomed) {
      stop();
      return;
    }

    if (Constants.ElevatorConstants.MIN_HEIGHT > desiredHeight) {
      desiredHeight = Constants.ElevatorConstants.MIN_HEIGHT;
    } else if (desiredHeight > Constants.ElevatorConstants.MAX_HEIGHT) {
      desiredHeight = Constants.ElevatorConstants.MAX_HEIGHT;
    }

    if (isAtTopLimit() && desiredHeight > getElevatorPosition()) {
      stop();
      return;
    }

    if (isAtBottomLimit() && desiredHeight < getElevatorPosition()) {
      stop();
      return;
    }

    double revs = heightToRevs(desiredHeight - currentHeight);
    elevatorMotor.setControl(motionMagicRequest.withPosition(revs));
  }

  public void moveElevator(double speed) {

    speed = Math.max(-1.0, Math.min(1.0, speed));
    if (!isHomed && speed > 0) {
      speed = 0;
    }

    if (isAtTopLimit() && speed > 0) {
      speed = 0;
    }

    if (isAtBottomLimit() && speed < 0) {
      speed = 0;
    }

    if (isHomed) {
      if (getElevatorPosition() >= Constants.ElevatorConstants.MAX_HEIGHT && speed > 0) {
        speed = 0;
      }

      if (getElevatorPosition() <= Constants.ElevatorConstants.MIN_HEIGHT && speed < 0) {
        speed = 0;
      }
    }

    elevatorMotor.set(speed);
  }

  public void HomeElevator(double speed) {
    elevatorMotor.set(speed);
  }

  // TODO log values of the limit switches, desired position, anything that would be helpful.
  public void zeroElevator() {
    elevatorMotor.stopMotor();
    elevatorMotor.setPosition(0);

    targetPosition = Constants.ElevatorConstants.MIN_HEIGHT;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    elevatorMotor.getConfigurator().apply(motorConfig.SoftwareLimitSwitch);
  }

  public boolean getIsHomed() {
    return isHomed;
  }

  public double getCurrentPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorPosition() {
    return revsToHeight(getCurrentPosition());
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public boolean atDesiredPosition(
      double desiredRotations, double errorRotations) { // TODO rename target postion to have meters
    return Math.abs(getCurrentPosition() - desiredRotations) <= errorRotations;
  }

  public boolean isAtTargetPosition() {
    return isHomed && Math.abs(getElevatorPosition() - targetPosition) <= Constants.ElevatorConstants.POSITION_TOLERANCE;
  }

  // check with build to see whether pressed is true or false
  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public boolean
      isAtTopLimit() { // double check how this is wired, if it is normally closed or normally open.
    // If it is normally closed, then this will be true when pressed.
    return !topLimitSwitch.get();
  }

  public void stop() {
    elevatorMotor.stopMotor();
  }

  private void updatePID() {

    Slot0Configs slot0 = motorConfig.Slot0;

    double kp = elevatorKp.get();
    double ki = elevatorKi.get();
    double kd = elevatorKd.get();

    double kg = elevatorKg.get();
    double ks = elevatorKs.get();
    double kv = elevatorKv.get();
    double ka = elevatorKa.get();

    if (kp != slot0.kP
        || ki != slot0.kI
        || kd != slot0.kD
        || kg != slot0.kG
        || ks != slot0.kS
        || kv != slot0.kV
        || ka != slot0.kA) {

      slot0.kP = kp;
      slot0.kI = ki;
      slot0.kD = kd;

      slot0.kG = kg;
      slot0.kS = ks;
      slot0.kV = kv;
      slot0.kA = ka;

      elevatorMotor.getConfigurator().apply(slot0);
    }
  }
}
