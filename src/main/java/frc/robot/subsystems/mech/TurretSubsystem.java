package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.logging.TalonFXLogger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;

  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private static Slot0Configs slot0Configs;
  private static MotionMagicConfigs motionMagicConfigs;
  private static CurrentLimitsConfigs turretCurrentLimitsConfigs;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 10.0;
  private boolean sysIdRunning = false;

  private final int TURRET_GEARBOX_RATIO = 1;

  /**
   * Called by SysId commands to indicate test is running; we log voltage/position/velocity in
   * periodic().
   */
  public void setSysIdRunning(boolean running) {
    sysIdRunning = running;
  }

  private final int GEAR_REVS_PER_TURRET_REV = 10;
  private final int ENCODER_REVS_PER_TURRET_REV = 10;
  private DutyCycleEncoder boreEncoder =
      new DutyCycleEncoder(TurretConstants.TURRET_BORE_ENCODER_PORT);

  private Rotation2d desiredAngle;

  // Tunable PID gains for turret
  public static final LoggedNetworkNumber turretKp =
      new LoggedNetworkNumber("/Tuning/Turret/kP", 4);
  public static final LoggedNetworkNumber turretKi =
      new LoggedNetworkNumber("/Tuning/Turret/kI", 0.0);
  public static final LoggedNetworkNumber turretKd =
      new LoggedNetworkNumber("/Tuning/Turret/kD", 0.1);

  public static final LoggedNetworkNumber turretKs =
      new LoggedNetworkNumber("/Tuning/Turret/kS", 0.25262);
  public static final LoggedNetworkNumber turretKv =
      new LoggedNetworkNumber("/Tuning/Turret/kV", 1.3679);
  public static final LoggedNetworkNumber turretKa =
      new LoggedNetworkNumber("/Tuning/Turret/kA", 0.3484);

  public TurretSubsystem() {
    turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredAngle = new Rotation2d(); // getCurrentAngle();

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    turretCurrentLimitsConfigs = talonFXConfigs.CurrentLimits;
    turretCurrentLimitsConfigs.StatorCurrentLimit = 80;
    turretCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

    slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = turretKs.get();
    slot0Configs.kV = turretKv.get();
    slot0Configs.kA = turretKa.get();

    slot0Configs.kP = turretKp.get();
    slot0Configs.kI = turretKi.get();
    slot0Configs.kD = turretKd.get();

    // MOTION MAGIC EXPO
    motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    turretMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
    // homeTurret();

    TalonFXLogger.configureTelemetryUpdateHz(turretMotor);
  }

  @Override
  public void periodic() {
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    // TODO: for any tuning of the turret, make sure to use this. once homing works without this,
    // bring it back and confirm it still works
    // updateSlot0Configs();

    // if (!sysIdRunning) {
    //   turretMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
    // }

    turretLogs();
  }

  // set the
  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    double desiredAngleDegrees =
        MathUtil.inputModulus(
            desiredAngle.getDegrees(),
            TurretConstants.MIN_TURRET_ANGLE,
            TurretConstants.MIN_TURRET_ANGLE + 360);
    double alternateDesiredAngleDegrees =
        MathUtil.inputModulus(
            desiredAngleDegrees,
            TurretConstants.MAX_TURRET_ANGLE - 360,
            TurretConstants.MAX_TURRET_ANGLE);
    if (Math.abs(desiredAngleDegrees - getCurrentAngle().getDegrees())
        < Math.abs(alternateDesiredAngleDegrees - getCurrentAngle().getDegrees())) {
      this.desiredAngle = new Rotation2d(Math.toRadians(desiredAngleDegrees));
    } else {
      this.desiredAngle = new Rotation2d(Math.toRadians(alternateDesiredAngleDegrees));
    }
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = turretMotor.getPosition().getValueAsDouble();
    double turretAngleDegrees =
        motorPositionRevs / TURRET_GEARBOX_RATIO / GEAR_REVS_PER_TURRET_REV * 360;
    return new Rotation2d(Math.toRadians(turretAngleDegrees));
  }

  public double degreesToRevs(double turretAngleDegrees) {
    return turretAngleDegrees / 360.0 * GEAR_REVS_PER_TURRET_REV * TURRET_GEARBOX_RATIO;
  }

  public void setMotorVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  private double getCurrentToOffsetError() {
    return boreEncoder.get() - TurretConstants.TURRET_ENCODER_OFFSET;
  }

  // sets both the current angle and the desired angle to 0
  public void homeTurret() {
    // TODO: try calling the other version of homeTurret in robotInit, my theory is
    // the bore encoder isn't connected by the time we call this
    // turretMotor.setPosition(0);
    turretMotor.setPosition(
        getCurrentToOffsetError()
                / ENCODER_REVS_PER_TURRET_REV
                * TURRET_GEARBOX_RATIO
                * GEAR_REVS_PER_TURRET_REV
            + degreesToRevs(TurretConstants.TURRET_HOMING_ANGLE));
    // System.out.println("ANGLE AT END OF TURRET HOMING: " + getCurrentAngle().getDegrees());
    setDesiredAngle((new Rotation2d(Math.toRadians(0))));
  }

  // get velocity in radians
  private double getVelocityRadPerSec() {
    double motorRPS = turretMotor.getVelocity().getValueAsDouble();
    return motorRPS / TURRET_GEARBOX_RATIO / GEAR_REVS_PER_TURRET_REV * 2 * Math.PI;
  }

  private SysIdRoutine sysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(1),
            // this is the maximum voltage for the test
            Volts.of(4),
            // this is the duration of the test.
            // Note we use `until` when we return the command to abort if we hit turret
            // limits
            Seconds.of(5),
            (state) -> Logger.recordOutput("Mech/Turret/SysID/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage; we log voltage/position/velocity ourselves in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> turretMotor.setVoltage(voltage.in(Volts)),
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "turret");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    boolean isSysIdOutOfBounds =
        angleDeg >= TurretConstants.MAX_TURRET_ANGLE - SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg <= TurretConstants.MIN_TURRET_ANGLE + SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Shooter/SysID/Outofbounds", isSysIdOutOfBounds);
    return isSysIdOutOfBounds;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .quasistatic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Turret SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .dynamic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Turret SysId Dynamic " + direction);
  }

  public void updateSlot0Configs() {
    double newKp = turretKp.get();
    double newKi = turretKi.get();
    double newKd = turretKd.get();
    double newKs = turretKs.get();
    double newKv = turretKv.get();
    double newKa = turretKa.get();
    if (newKp != slot0Configs.kP
        || newKi != slot0Configs.kI
        || newKd != slot0Configs.kD
        || newKa != slot0Configs.kA
        || newKv != slot0Configs.kV
        || newKs != slot0Configs.kS) {
      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      slot0Configs.kS = newKs;
      slot0Configs.kV = newKv;
      slot0Configs.kA = newKa;
      turretMotor.getConfigurator().apply(talonFXConfigs);
    }
  }

  public void turretLogs() {
    TalonFXLogger.log(turretMotor, "Mech", "Turret");
    Logger.recordOutput("Mech/Turret/currentAngle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Turret/desiredAngle", desiredAngle.getDegrees());

    Logger.recordOutput("Mech/Turret/Encoder/boreEncoder", boreEncoder.get());
    Logger.recordOutput("Mech/Turret/Encoder/boreEncoder isConnected", boreEncoder.isConnected());
    Logger.recordOutput("Mech/Turret/Encoder/get bore offset", getCurrentToOffsetError());

    Logger.recordOutput(
        "Mech/Turret/what home turret thinks its doing",
        getCurrentToOffsetError()
                / ENCODER_REVS_PER_TURRET_REV
                * TURRET_GEARBOX_RATIO
                * GEAR_REVS_PER_TURRET_REV
            + degreesToRevs(TurretConstants.TURRET_HOMING_ANGLE));

    // SysID
    Logger.recordOutput("Mech/Turret/SysID/turretSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput(
          "Mech/Turret/SysID/turretVoltage", turretMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Turret/SysID/turretPosition",
          getCurrentAngle().getRadians() / (2.0 * Math.PI)); // rotations
      Logger.recordOutput(
          "Mech/Turret/SysID/turretVelocity", getVelocityRadPerSec() / (2.0 * Math.PI)); // rot/s
    }
  }
}
