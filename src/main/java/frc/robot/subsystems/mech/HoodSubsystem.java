package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.logging.TalonFXLogger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodSubsystem extends SubsystemBase {

  // when false, run voltage control
  private boolean positionControl = true;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 2;

  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  private TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private boolean sysIdRunning = false;
  private SysIdRoutine sysIdRoutine;
  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0);
  private Rotation2d desiredAngle = getCurrentAngle();

  private static final double HOOD_CURRENT_LIMIT = 60; // TODO change

  // Tunable PID gains for hood control
  public static final LoggedNetworkNumber hoodKp = new LoggedNetworkNumber("/Tuning/Hood/kP", 30);
  public static final LoggedNetworkNumber hoodKi = new LoggedNetworkNumber("/Tuning/Hood/kI", 0.0);
  public static final LoggedNetworkNumber hoodKd = new LoggedNetworkNumber("/Tuning/Hood/kD", 0.1);

  public static final LoggedNetworkNumber hoodKg = new LoggedNetworkNumber("/Tuning/Hood/kG", 0.2);
  public static final LoggedNetworkNumber hoodKs = new LoggedNetworkNumber("/Tuning/Hood/kS", 0.01);
  public static final LoggedNetworkNumber hoodKv = new LoggedNetworkNumber("/Tuning/Hood/kV", 0.16);
  public static final LoggedNetworkNumber hoodKa = new LoggedNetworkNumber("/Tuning/Hood/kA", 0.01);

  public static final LoggedNetworkNumber hoodMMKv =
      new LoggedNetworkNumber("/Tuning/Hood/MM kV", 0.16);
  public static final LoggedNetworkNumber hoodMMKa =
      new LoggedNetworkNumber("/Tuning/Hood/MM kA", 0.1);
  public static final LoggedNetworkNumber tunableHoodAngle =
      new LoggedNetworkNumber(
          "/Tuning/Hood/tunableHoodAngle", HoodConstants.RETRACTED_POSITION.getDegrees());

  public HoodSubsystem() {
    // MOTION MAGIC PID/FEEDFORWARD CONFIGS
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG = hoodKg.get(); // Add ____V output to overcome gravity
    slot0Configs.kS = hoodKs.get(); // Add _____ V output to overcome static friction

    slot0Configs.kV = hoodKv.get(); // Output per unit of target velocity (output/rps)
    slot0Configs.kA = hoodKa.get(); // Output per unit of target (output/(rps/s))

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = hoodKp.get(); // Output per unit of error in position (output/rotation)
    slot0Configs.kI =
        hoodKi.get(); // Output per unit of integrated error in position (output/(rotation*s))
    slot0Configs.kD = hoodKd.get(); // Output per unit of error in velocity (output/rps)

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // when 0, unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV =
        0.16; // Voltage required to apply a given acceleration (V/(rps/s))
    motionMagicConfigs.MotionMagicExpo_kA =
        0.2; // Voltage required to maintain a given velocity (V/rps)

    hoodMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    // desiredAngle = new Rotation2d(Math.toRadians(tunableHoodAngle.get()));
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    double newKp = hoodKp.get();
    double newKi = hoodKi.get();
    double newKd = hoodKd.get();

    double newKg = hoodKg.get();
    double newKs = hoodKs.get();
    double newKv = hoodKv.get();
    double newKa = hoodKa.get();

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

      hoodMotor.getConfigurator().apply(talonFXConfigs);
    }

    // Skip current switch safety during SysID - the isSysIdOutOfBounds() handles limits

    /* tells it to stop if using position and at limitswitch
        if by speed and not position, also tells to stop

    if the current limit is triggered and sysId is not running
     *   if the hood is being controlled by position
     *      if the desired angle is greater than the current angle
     *         set the desired angle to the current angle
     *         set the hood speed to 0
     *   if hood is not being controlled by position
     *      if hood speed is greater than 0
     *        set the hood speed to 0
     */
    if (isCurrentLimitReached() && !sysIdRunning) {
      if (positionControl) {
        if (desiredAngle.getDegrees() > getCurrentAngle().getDegrees()) {
          desiredAngle = getCurrentAngle();
          setHoodSpeed(0);
        }
      } else {
        if (hoodMotor.getMotorVoltage().getValueAsDouble() > 0) {
          setHoodSpeed(0);
        }
      }
    }

    // Only run position control if SysID is not running
    if (positionControl && !sysIdRunning) {
      setHoodPosition(desiredAngle);
    }

    hoodLogs();
  }

  // sets the desired angle for the hood, caps within bounds
  public void setDesiredAngle(Rotation2d desiredAngle) {
    positionControl = true;
    if (desiredAngle.getDegrees() > HoodConstants.RETRACTED_POSITION.getDegrees()) {
      desiredAngle = HoodConstants.RETRACTED_POSITION;
    }
    if (desiredAngle.getDegrees() < HoodConstants.MIN_ANGLE.getDegrees()) {
      desiredAngle = HoodConstants.MIN_ANGLE;
    }
    this.desiredAngle = desiredAngle;
  }

  // gets the desired angle
  public Rotation2d getDesiredAngle() {
    return desiredAngle;
  }

  // sets the current hood position (resets)
  private void setHoodPosition(Rotation2d desiredAngle) {
    positionControl = true;
    hoodMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
  }

  // angle that the ball is coming out of the hood at (an offset for hood angle)
  public static Rotation2d launchAngleToHoodAngle(Rotation2d launchAngle) {
    return launchAngle;
  }

  // convert from degrees to revolutions for the hood gear
  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees
        / 360.0
        * HoodConstants.HOOD_SHAFT_REVS_PER_MECH_REV
        * HoodConstants.HOOD_GEAR_RATIO;
  }

  // get the current angle of the hood
  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = hoodMotor.getPosition().getValueAsDouble();
    double hoodAngleDegrees =
        motorPositionRevs
            / HoodConstants.HOOD_GEAR_RATIO
            / HoodConstants.HOOD_SHAFT_REVS_PER_MECH_REV
            * 360
            % 360;
    return new Rotation2d(Math.toRadians(hoodAngleDegrees));
  }

  // set the speed of the hood
  public void setHoodSpeed(double velocity) {
    positionControl = false;
    hoodMotor.set(velocity);
  }

  // returns true if the current limit is reached
  public boolean isCurrentLimitReached() {
    if (hoodMotor.getStatorCurrent().getValueAsDouble() > HOOD_CURRENT_LIMIT) {
      return true;
    }
    return false;
  }

  // sets the hood position to 0
  public void zeroHood() {
    hoodMotor.setPosition(degreesToRevs(HoodConstants.RETRACTED_POSITION.getDegrees()));
    setDesiredAngle(HoodConstants.RETRACTED_POSITION);
  }

  // get the current velocity in radians per second
  private double getVelocityRadPerSec() {
    double motorRPS = hoodMotor.getVelocity().getValueAsDouble();
    return motorRPS
        / HoodConstants.HOOD_GEAR_RATIO
        / HoodConstants.HOOD_SHAFT_REVS_PER_MECH_REV
        * 2
        * Math.PI;
  }

  // set the hood to be controlled by position or not (other option is by velocity)
  public void setPositionControl(boolean positionControl) {
    this.positionControl = positionControl;
  }

  private void initSysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(2),
            // this is the maximum voltage for the test
            Volts.of(1.5),
            // this is the duration of the test.
            Seconds.of(1.5),
            (state) -> Logger.recordOutput("Mech/Hood/SysID/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage; we log voltage/position/velocity ourselves in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              sysIdRunning = true;
              hoodMotor.setControl(sysIdVoltageRequest.withOutput(voltage.in(Volts)));
            },
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "hood");
    // System.out.println("CREATING NEW SYSID ROUTINE");
    sysIdRoutine = new SysIdRoutine(config, mechanism);
  }

  // tests if outside of the sysId boundaries
  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    Logger.recordOutput("Mech/Hood/SysID/SysID Current Angle", angleDeg);
    boolean outOfBounds =
        angleDeg > HoodConstants.RETRACTED_POSITION.getDegrees() + SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg < HoodConstants.MIN_ANGLE.getDegrees() - SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Hood/SysID/SysId Out Of Bounds", outOfBounds);
    Logger.recordOutput(
        "Mech/Hood/SysID/SysId Retracted Limit",
        HoodConstants.RETRACTED_POSITION.getDegrees() + SYSID_LIMIT_MARGIN_DEGREES);
    Logger.recordOutput(
        "Mech/Hood/SysID/SysId Min Limit",
        HoodConstants.MIN_ANGLE.getDegrees() - SYSID_LIMIT_MARGIN_DEGREES);
    return outOfBounds;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // System.out.println("RUNNING SYSID QUASISTATIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .quasistatic(direction)
        .until(this::isSysIdOutOfBounds) // temporarily disabled for testing
        .finallyDo(
            () -> {
              setHoodSpeed(0);
              sysIdRunning = false;
            })
        .withName("Hood SysId Quasistatic " + direction);
  }

  // measure acceleration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // System.out.println("RUNNING SYSID DYNAMIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .dynamic(direction)
        .until(this::isSysIdOutOfBounds) // temporarily disabled for testing
        .finallyDo(
            () -> {
              setHoodSpeed(0);
              sysIdRunning = false;
            })
        .withName("Hood SysId Dynamic " + direction);
  }

  // logs various things for hood (desired angle, angle, current limit, etc.)
  public void hoodLogs() {
    TalonFXLogger.log(hoodMotor, "Mech", "Hood");

    Logger.recordOutput("Mech/Hood/Desired Angle", desiredAngle.getDegrees());
    Logger.recordOutput("Mech/Hood/Current Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Hood/Current Limit Reached", isCurrentLimitReached());
    Logger.recordOutput(
        "Mech/Hood/Control Mode", positionControl ? "position control" : "voltage control");

    // SysID
    Logger.recordOutput("Mech/Hood/SysID/hoodSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput(
          "Mech/Hood/SysID/hoodVoltage", hoodMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Hood/SysID/hoodPosition",
          getCurrentAngle().getRadians() / (2.0 * Math.PI)); // rotations
      Logger.recordOutput(
          "Mech/Hood/SysID/hoodVelocity", getVelocityRadPerSec() / (2.0 * Math.PI)); // rot/s
    }
  }
}
