// TODO intended to start pitch at loc 0

package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController; // pid (converts position to voltage)
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private double rollKP = 0;
  private double rollKI = 0;
  private double rollKD = 0;
  private PIDController rollPID = new PIDController(rollKP, rollKI, rollKD);
  private double pitchKP = 0;
  private double pitchKI = 0;
  private double pitchKD = 0;
  private PIDController pitchPID = new PIDController(pitchKP, pitchKI, pitchKD);
  private TalonFX leftMotor = new TalonFX(Constants.WristConstants.LEFT_WRIST_MOTOR_ID);
  private TalonFX rightMotor = new TalonFX(Constants.WristConstants.RIGHT_WRIST_MOTOR_ID);
  private Rotation2d pitchPos = new Rotation2d();
  private Rotation2d rollPos = new Rotation2d();

  public WristSubsystem() {
    TalonFXConfigurator leftMotorConfigtor = leftMotor.getConfigurator();
    TalonFXConfigurator rightMotorConfigtor = rightMotor.getConfigurator();
    TalonFXConfiguration leftMotorConfigtion = new TalonFXConfiguration();
    TalonFXConfiguration rightMotorConfigtion = new TalonFXConfiguration();
    leftMotorConfigtion.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfigtion.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotorConfigtor.apply(leftMotorConfigtion);
    rightMotorConfigtor.apply(rightMotorConfigtion);
    // motor configs(check w/ build with motor inversions) TODO: Motor inversions
  }

  @Override
  public void periodic() {
    pitchPos = Rotation2d.fromDegrees(rightMotor.getPosition().getValueAsDouble());
    rollPos = Rotation2d.fromDegrees(leftMotor.getPosition().getValueAsDouble());
  }

  public void setPostion(Rotation2d pitchPos, Rotation2d rollPos) {
    double pitchVoltage = pitchPID.calculate(pitchPos.getDegrees());
    double rollVoltage = rollPID.calculate(rollPos.getDegrees());
    double leftVoltage = pitchVoltage - rollVoltage;
    double rightVoltage = pitchVoltage + rollVoltage;
    leftVoltage =
        MathUtil.clamp(
            leftVoltage,
            Constants.WristConstants.VOLTAGE_REVERSE_MAX,
            Constants.WristConstants.VOLTAGE_MAX);
    rightVoltage =
        MathUtil.clamp(
            rightVoltage,
            Constants.WristConstants.VOLTAGE_REVERSE_MAX,
            Constants.WristConstants.VOLTAGE_MAX);
    setVoltage(leftVoltage, rightVoltage);
  }

  public void setZero() {}

  public void setVoltage(double leftVoltage, double rightVoltage) {
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }
}
