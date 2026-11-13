/* TODO (general todos)
 * figure out our units for position and adjust the data type accordingly
 * position and speed control
 */

package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// TODO imports (including motor controllers/encoders)
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController; // pid (converts position to voltage)
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WristSubsystem extends SubsystemBase {
  // The differential wrist (might help to look at a picture online) consists of two motors.
  // If they spin in the same direction, the wrist moves up and down. If they spin in opposite
  // directions, the wrist rotates.
  private TalonFX rightWristMotor;
  private TalonFX leftWristMotor;
  private PIDController pid;
  private Rotation2d currentRotationPosition; // we plan to use degrees as our unit
  private Rotation2d currentPivotPosition; // we plan to use degrees as our unit

  private final TalonFXConfiguration leftTalonFXConfigs;
  private final TalonFXConfiguration rightTalonFXConfigs;
  private static CurrentLimitsConfigs leftCurrentLimitConfigs;
  private static CurrentLimitsConfigs rightCurrentLimitConfigs;

  private final LoggedNetworkNumber wristkP = new LoggedNetworkNumber("", 0.1);
  private final LoggedNetworkNumber wristkI = new LoggedNetworkNumber("", 0.01);
  private final LoggedNetworkNumber wristkD = new LoggedNetworkNumber("", 0.001);

  public WristSubsystem() {
    rightWristMotor = new TalonFX(8, Constants.TunerConstants.driveCANBus);
    leftWristMotor = new TalonFX(9, Constants.TunerConstants.driveCANBus);
    // TODO find IDs
    // TODO find what we want the current limits to actally be

    // TODO: Create motor configurations
    leftTalonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    rightTalonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    leftCurrentLimitConfigs = leftTalonFXConfigs.CurrentLimits;
    leftCurrentLimitConfigs.StatorCurrentLimit = 70;
    leftCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    rightCurrentLimitConfigs = rightTalonFXConfigs.CurrentLimits;
    rightCurrentLimitConfigs.StatorCurrentLimit = 70;
    rightCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    leftWristMotor.getConfigurator().apply(leftTalonFXConfigs);
    rightWristMotor.getConfigurator().apply(rightTalonFXConfigs);

    // TODO: Configure PID gains, current limits, other safety things, etc
    pid = new PIDController(0.1, 0.01, 0.001); // Placeholder PID gains, created and set PID
    // TODO ki, kp, and kd should be tunables
    currentPivotPosition = Rotation2d.fromDegrees(0);
    currentRotationPosition = Rotation2d.fromDegrees(0);

    // TODO: Ask build about inversions of motors
  }

  public void periodic() {}

  // TODO other useful methods

  public void setVoltage(double rightMotorVoltage, double leftMotorVoltage) { // set two voltages
    rightWristMotor.setVoltage(rightMotorVoltage);
    leftWristMotor.setVoltage(leftMotorVoltage);
  }

  public void setRotation(Rotation2d desiredPosition) {
    double voltage =
        pid.calculate(
            currentRotationPosition.getDegrees(),
            desiredPosition
                .getDegrees()); // calculate voltage based on current and desired position
    setVoltage(-voltage, voltage); // rotates (NOT flip over)
  }

  public void setPivot(Rotation2d desiredPosition) {
    double voltage =
        pid.calculate(
            currentPivotPosition.getDegrees(),
            desiredPosition
                .getDegrees()); // calculate voltage based on current and desired position
    setVoltage(voltage, voltage); // fliping it on the z-axis
  }

  public double getRightWristVoltage() {
    return rightWristMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getLeftWristVoltage() {
    return leftWristMotor.getMotorVoltage().getValueAsDouble();
  }
  


  public Rotation2d getPivot() { // TODO write method to get position
    return new Rotation2d();
    // TODO write degrees to revs method
  }
  // to find required gear ratio for motors we need to do trial and error :(
}
