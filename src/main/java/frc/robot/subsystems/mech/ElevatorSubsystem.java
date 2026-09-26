package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(1, "rio"); // CAN ID currently placeholder value
  private TalonFXConfiguration talonFXConfigs; // allows us to store tuning constants
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public static final LoggedNetworkNumber elevatorKp =
      new LoggedNetworkNumber("/Tuning/Elevator/kP", 30);
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

  public ElevatorSubsystem() {

    talonFXConfigs = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    motionMagicConfigs.MotionMagicCruiseVelocity = 100.0; // target maximum velocity (rotations/sec)
    motionMagicConfigs.MotionMagicAcceleration =
        40.0; // target maximum acceleration (rotations/sec^2)
    motionMagicConfigs.MotionMagicJerk =
        400.0; // target maximum acceleration (rotations/sec^3), or how fast it can change
    // acceleration

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = elevatorKp.get(); // Output per unit of error in position (output/rotation)
    slot0Configs.kI =
        elevatorKi.get(); // Output per unit of integrated error in position (output/(rotation*s))
    slot0Configs.kD = elevatorKd.get(); // Output per unit of error in velocity (output/rps)

    slot0Configs.kG = elevatorKg.get(); // Add ____V output to overcome gravity
    slot0Configs.kS = elevatorKs.get(); // Add _____ V output to overcome static friction

    slot0Configs.kV = elevatorKv.get(); // Output per unit of target velocity (output/rps)
    slot0Configs.kA = elevatorKa.get(); // Output per unit of target (output/(rps/s))

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public void setTargetPosition(double targetRotations) {
    //TODO setControl() doesn't work if you just call it once to move somewhere, you have to call it repeatedly
    motor.setControl(m_request.withPosition(targetRotations));
  }

  public double getCurrentPosition() {
    //TODO wow i had never heard of the refresh method, thx for the new info
    return motor.getPosition().refresh().getValueAsDouble();
  }

  public boolean hasReachedTarget(double targetRotations) {
    return Math.abs(getCurrentPosition() - targetRotations)
        <= 0.01; // allowed error currently placeholder value
  }

  public double getVelocity() {
    return motor.getVelocity().refresh().getValueAsDouble();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void periodic() {

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    // new values updated in logged network
    double newKp = elevatorKp.get();
    double newKi = elevatorKi.get();
    double newKd = elevatorKd.get();

    double newKg = elevatorKg.get();
    double newKs = elevatorKs.get();
    double newKv = elevatorKv.get();
    double newKa = elevatorKa.get();

    if (newKp != slot0Configs.kP
        || newKi != slot0Configs.kI
        || newKd != slot0Configs.kD
        || newKg != slot0Configs.kG
        || newKs != slot0Configs.kS
        || newKv != slot0Configs.kV
        || newKa != slot0Configs.kA) {

      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      slot0Configs.kG = newKg;
      slot0Configs.kS = newKs;
      slot0Configs.kV = newKv;
      slot0Configs.kA = newKa;

      motor.getConfigurator().apply(slot0Configs);
    }
  }
}
