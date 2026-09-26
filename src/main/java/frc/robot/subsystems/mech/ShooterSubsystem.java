package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConditions;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.logging.TalonFXLogger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotor;
  private final TalonFX kickerMotor;
  private final TalonFX leftTransitionMotor;
  private final TalonFX rightTransitionMotor;

  private double desiredTransitionSpeed;

  private static MotionMagicVelocityVoltage m_request;

  private static TalonFXConfiguration flywheelTalonFXConfigs;
  private static TalonFXConfiguration transitionMotorConfigs;
  private static TalonFXConfiguration leftTransitionMotorConfigs;
  private static TalonFXConfiguration rightTransitionMotorConfigs;

  private static Slot0Configs flywheelSlot0Configs;
  private static MotionMagicConfigs flywheelMotionMagicConfigs;

  private boolean sysIdRunning = false;
  private SysIdRoutine sysIdRoutine;
  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0);
  private final NeutralOut flywheelNeutralOut = new NeutralOut();

  // private LoggedNetworkNumber desiredRotorVelo =
  //     new LoggedNetworkNumber("Mech/Shooter/Desired Rotor Velo", 53);
  private double desiredRotorVelocity = 0;

  public static LoggedNetworkNumber flyWheelSlip =
      new LoggedNetworkNumber("/Tuning/Shooter/flywheelSlip", 0.255);

  // Tunable PID gains flywheel
  public static final LoggedNetworkNumber flywheelKP =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kP", 0.02);
  public static final LoggedNetworkNumber flywheelKI =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kI", 0.0);
  public static final LoggedNetworkNumber flywheelKD =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kD", 0.002);

  public ShooterSubsystem() {
    flywheelMotor =
        new TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    kickerMotor = new TalonFX(ShooterConstants.TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    leftTransitionMotor =
        new TalonFX(ShooterConstants.LEFT_TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    rightTransitionMotor =
        new TalonFX(ShooterConstants.RIGHT_TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    // TALONFX & MOTIONMAGIC CONFIGS
    flywheelTalonFXConfigs = new TalonFXConfiguration();

    flywheelTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));

    flywheelSlot0Configs = flywheelTalonFXConfigs.Slot0;

    // look at HoodSubsystem for explanations :)
    flywheelSlot0Configs.kS = 0.37577; // Add __ V output to overcome static friction
    flywheelSlot0Configs.kV = 0.12289; // A velocity target of 1 rps results in 0.12-0.2 V output
    flywheelSlot0Configs.kA = 0.023452;
    flywheelSlot0Configs.kP = flywheelKP.get();
    flywheelSlot0Configs.kI = flywheelKI.get();
    flywheelSlot0Configs.kD = flywheelKD.get();

    // MOTION MAGIC EXPO
    flywheelMotionMagicConfigs = flywheelTalonFXConfigs.MotionMagic;

    flywheelMotionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    flywheelMotionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0/1 seconds)

    // flywheelTalonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;

    transitionMotorConfigs = new TalonFXConfiguration();

    transitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    leftTransitionMotorConfigs = new TalonFXConfiguration();

    leftTransitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    rightTransitionMotorConfigs = new TalonFXConfiguration();

    rightTransitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    transitionMotorConfigs.CurrentLimits.StatorCurrentLimit = 80;
    applyTransitionMotorCurrentLimits(leftTransitionMotorConfigs);
    applyTransitionMotorCurrentLimits(rightTransitionMotorConfigs);

    flywheelMotor.getConfigurator().apply(flywheelTalonFXConfigs);
    kickerMotor.getConfigurator().apply(transitionMotorConfigs);
    leftTransitionMotor.getConfigurator().apply(leftTransitionMotorConfigs);
    rightTransitionMotor.getConfigurator().apply(rightTransitionMotorConfigs);

    m_request = new MotionMagicVelocityVoltage(0);
  }

  @Override
  public void periodic() {
    updateSlot0Configs();

    // Only control motors if SysID is not running
    if (!sysIdRunning) {
      if (desiredRotorVelocity == 0.0) { // stops if desired velocity is 0
        flywheelMotor.setControl(flywheelNeutralOut);
      } else { // sets speed to desired velocity
        flywheelMotor.setControl(m_request.withVelocity(desiredRotorVelocity));
      }
    }

    kickerMotor.set(desiredTransitionSpeed * 2.0); // originally *2.5
    leftTransitionMotor.set(desiredTransitionSpeed * 1.5);
    rightTransitionMotor.set(desiredTransitionSpeed * 1.5);

    shooterLogs();
  }

  public void setDesiredRotorVelocity(double desiredRotorVelocity) {
    this.desiredRotorVelocity = desiredRotorVelocity;
  }

  public double getFlywheelRotorVelocity() {
    return flywheelMotor.getRotorVelocity().getValueAsDouble();
  }

  public void setDesiredTransitionSpeed(double desiredTransitionSpeed) {
    this.desiredTransitionSpeed = desiredTransitionSpeed;
  }

  // how fast the ball is leaving meters/sec
  public double getExitVelocity() {
    return flyWheelSlip.get()
        * getFlywheelRotorVelocity()
        * 2
        * Math.PI
        * ShooterConstants.FLYWHEEL_RADIUS_METERS
        * ShooterConstants.FLYWHEEL_GEAR_RATIO;
  }

  // shotSpeed in meters/second
  public static double launchSpeedToRotorSpeed(double shotSpeed) {
    return shotSpeed
        / ShooterConstants.FLYWHEEL_GEAR_RATIO
        / 2
        / Math.PI
        / ShooterConstants.FLYWHEEL_RADIUS_METERS
        / flyWheelSlip.get();
  }

  private void initSysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(1),
            // this is the maximum voltage for the test
            Volts.of(14),
            // this is the duration of the test.
            Seconds.of(15),
            (state) -> Logger.recordOutput("Mech/Shooter/SysID/SysIdState", state.toString()));

    // mechanism for our test. Drives both flywheels; we log voltage/position/velocity for each in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              sysIdRunning = true;
              // leftFlywheelMotor.setControl(sysIdVoltageRequest.withOutput(voltage.in(Volts)));
              flywheelMotor.setControl(sysIdVoltageRequest.withOutput(voltage.in(Volts)));
            },
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "shooter");
    // System.out.println("CREATING NEW SYSID ROUTINE");
    sysIdRoutine = new SysIdRoutine(config, mechanism);
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // System.out.println("RUNNING SYSID QUASISTATIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .quasistatic(direction)
        .finallyDo(() -> sysIdRunning = false)
        .withName("Flywheel SysId Quasistatic " + direction);
  }

  // measure acceleration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // System.out.println("RUNNING SYSID DYNAMIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .dynamic(direction)
        .finallyDo(() -> sysIdRunning = false)
        .withName("Flywheel SysId Dynamic " + direction);
  }

  public void shooterLogs() {
    TalonFXLogger.log(flywheelMotor, "Mech", "Shooter", "Flywheel");
    Logger.recordOutput(
        "Mech/Shooter/Flywheel/Flywheel Rotor Velocity", getFlywheelRotorVelocity());
    Logger.recordOutput("Mech/Shooter/Flywheel/Desired Rotor Velocity", desiredRotorVelocity);

    Logger.recordOutput("Mech/Shooter/Desired Transition Speed", desiredTransitionSpeed);

    TalonFXLogger.log(kickerMotor, "Mech", "Shooter", "Kicker");
    Logger.recordOutput(
        "Mech/Shooter/Kicker/Kicker Running",
        (kickerMotor.getMotorVoltage().getValueAsDouble() != 0));

    TalonFXLogger.log(leftTransitionMotor, "Mech", "Shooter", "Left Transition");
    TalonFXLogger.log(rightTransitionMotor, "Mech", "Shooter", "Right Transition");

    Logger.recordOutput("Mech/Shooter/MAX SPEED", ShotCalculatorConditions.MAX_SHOT_SPEED);
    Logger.recordOutput(
        "Mech/Shooter/MAX ROTOR SPEED",
        launchSpeedToRotorSpeed(ShotCalculatorConditions.MAX_SHOT_SPEED));

    // SysID
    Logger.recordOutput("Mech/Shooter/SysID/shooterSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      // Left flywheel
      Logger.recordOutput(
          "Mech/Shooter/SysID/leftShooterVoltage",
          flywheelMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Shooter/SysID/leftShooterPosition",
          flywheelMotor.getPosition().getValueAsDouble()); // rotations
      Logger.recordOutput(
          "Mech/Shooter/SysID/leftShooterVelocity",
          flywheelMotor.getVelocity().getValueAsDouble()); // output rot/s
    }
  }

  private static void applyTransitionMotorCurrentLimits(TalonFXConfiguration config) {
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.TRANSITION_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
  }

  public void updateSlot0Configs() {
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    double newFlywheelKP = flywheelKP.get();
    double newFlywheelKI = flywheelKI.get();
    double newFlywheelKD = flywheelKD.get();
    if (newFlywheelKP != flywheelSlot0Configs.kP
        || newFlywheelKI != flywheelSlot0Configs.kI
        || newFlywheelKD != flywheelSlot0Configs.kD) {
      flywheelSlot0Configs.kP = newFlywheelKP;
      flywheelSlot0Configs.kI = newFlywheelKI;
      flywheelSlot0Configs.kD = newFlywheelKD;
      flywheelMotor.getConfigurator().apply(flywheelTalonFXConfigs);
    }
  }

  // Shoot as far as possible
  public void runFarthestShot() {
    setDesiredRotorVelocity(90); // set velocity to our desired velocity
    if (Math.abs(getFlywheelRotorVelocity() - 90)
        < ShooterConstants
            .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired velocity
      setDesiredTransitionSpeed(ShooterConstants.TRANSITION_SPEED);
    }
  }
}
