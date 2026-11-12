package frc.robot.subsystems.mech;

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
import frc.robot.Constants.ElevatorConstants;

// Selin's grain of salt is that we should be using inches throughout this subsystem 
// make the move position into a command and a stop command in one file
// figure out what the hell homing is on
// remove elevator constants cuz merge conflicts
// merge when things work
public class ElevatorSubsystem extends SubsystemBase {
  // TODO put constants in constants.java file
  private static final int MOTOR_ID = 0; // TODO: Verify CAN ID
  private static final double GEAR_RATIO = 5.0;
  private static final double DRUM_CIRCUMFERENCE =
      0.1; // TODO: Replace with actual drum circumference
  private static final double MIN_HEIGHT = 0.0;
  private static final double MAX_HEIGHT = 1.5; // TODO: Replace with actual maximum height
  private static final double POSITION_TOLERANCE = 0.02;
  private static final double currentHeight = 0.0; // TODO: Replace with actual current height logic/ homing things when the time is nigh
  // TODO: Verify motor direction.
  private static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  private final TalonFX elevatorMotor = new TalonFX(MOTOR_ID);

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

    motorConfig.MotorOutput.Inverted = MOTOR_DIRECTION;
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

    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = metersToMotorPosition(MIN_HEIGHT);

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
  }

  private double metersToMotorPosition(double meters) {
    return (meters / DRUM_CIRCUMFERENCE) * GEAR_RATIO; // TODO what is drum circumfrence?
  }

  private double heightToRevs(double height) { // height is in inches, returns revs of the motor
    return height / ElevatorConstants.HEIGHT_TO_REVS;
  }

  private double revsToHeight(double revs) { // revs of the motor, returns height in inches
    return revs * ElevatorConstants.HEIGHT_TO_REVS;
  }

  private void goToHeight(double currentHeight, double desiredHeight) { // height is in inches
    double revs = heightToRevs(desiredHeight - currentHeight);
    elevatorMotor.setControl(motionMagicRequest.withPosition(revs));
  }

  private double motorPositionToMeters(double rotations) {
    return (rotations / GEAR_RATIO) * DRUM_CIRCUMFERENCE; // position is unclear- just watch out for
  }

  public void moveToPosition(double currentHeight, double desiredHeight){ // height is in inches
    if (!isHomed) {
      stop();
      return;
    }

    if (MIN_HEIGHT > desiredHeight) {
      desiredHeight = MIN_HEIGHT;
    } else if (desiredHeight > MAX_HEIGHT) {
      desiredHeight = MAX_HEIGHT;
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

  public void setDesiredPosition(double rotations) { // This method makes me sad. why is the input revs when we can just use height?
    moveToPosition((rotations),currentHeight);
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
      if (getElevatorPosition() >= MAX_HEIGHT && speed > 0) {
        speed = 0;
      }

      if (getElevatorPosition() <= MIN_HEIGHT && speed < 0) {
        speed = 0;
      }
    }

    elevatorMotor.set(speed);
  }

  // TODO log values of the limit switches, desired position, anything that would be helpful.
  public void zeroElevator() {

    if (!isAtBottomLimit()) {
      return;
    }

    elevatorMotor.stopMotor();
    elevatorMotor.setPosition(0.0);

    targetPosition = MIN_HEIGHT;
    isHomed = true;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    elevatorMotor.getConfigurator().apply(motorConfig.SoftwareLimitSwitch);
  }

  public boolean isHomed() {
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
    return isHomed && Math.abs(getElevatorPosition() - targetPosition) <= POSITION_TOLERANCE;
  }

  // check with build to see whether pressed is true or false
  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public boolean isAtTopLimit() { // double check how this is wired, if it is normally closed or normally open. If it is normally closed, then this will be true when pressed.
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
