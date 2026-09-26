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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.logging.TalonFXLogger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor; // spins the rollers
  private final TalonFX deployMotor; // deploys the entire intake

  private final DigitalInput retractedLimitSwitch;
  private final DigitalInput deployedLimitSwitch;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 3;

  public enum IntakeDeployState {
    SYSID,
    DEPLOYING,
    RETRACTING,
    DEPLOYED_STOPPED,
    RETRACTED_STOPPED
  }

  private IntakeDeployState intakeState;

  private final TalonFXConfiguration deployTalonFXConfigs;
  private final TalonFXConfiguration intakeTalonFXConfigs;

  private static Slot0Configs slot0Configs;
  private static MotionMagicConfigs motionMagicConfigs;
  private static CurrentLimitsConfigs deployCurrentLimitConfigs;
  private static CurrentLimitsConfigs intakeCurrentLimitConfigs;

  /**
   * Called by SysId commands to indicate test is running; we log voltage/position/velocity in
   * periodic().
   */
  public void setSysIdRunning(boolean running) {
    intakeState = IntakeDeployState.SYSID;
  }

  private static MotionMagicExpoVoltage m_request;

  // Tunable PID gains for intake deploy
  public static final LoggedNetworkNumber intakeKp =
      new LoggedNetworkNumber("/Tuning/Intake/kP", 6);
  public static final LoggedNetworkNumber intakeKi =
      new LoggedNetworkNumber("/Tuning/Intake/kI", 0.0);
  public static final LoggedNetworkNumber intakeKd =
      new LoggedNetworkNumber("/Tuning/Intake/kD", 0.1);

  // Tunable Feedfoward gains for intake deploy
  public static final LoggedNetworkNumber intakeKg =
      new LoggedNetworkNumber("/Tuning/Intake/kG", 0.0);
  public static final LoggedNetworkNumber intakeKs =
      new LoggedNetworkNumber("/Tuning/Intake/kS", 0.0);
  public static final LoggedNetworkNumber intakeKv =
      new LoggedNetworkNumber("/Tuning/Intake/kV", 0.16);
  public static final LoggedNetworkNumber intakeKa =
      new LoggedNetworkNumber("/Tuning/Intake/kA", 0.01);

  public static final LoggedNetworkNumber intakeExpoKa =
      new LoggedNetworkNumber("/Tuning/Intake/Expo kA", 0.16);
  public static final LoggedNetworkNumber intakeExpoKv =
      new LoggedNetworkNumber("/Tuning/Intake/Expo kV", 0.1);

  public static final LoggedNetworkNumber intakeCurrentLimit =
      new LoggedNetworkNumber("/Tuning/Intake/Intake Current Limit", 25);

  private boolean useDeployPositionControl = false;
  private double desiredIntakeSpeed;
  private double desiredDeploySpeed;
  private BooleanSupplier isDeployed;

  private boolean deployManualControl = false;
  private double deployManualSpeed = 0.0;
  private boolean deployGoalExtended = false;

  /** True while periodic is driving deploy open-loop to find a hall (not a command). */
  private boolean hallAssistActive = false;

  /**
   * Tracks open-loop hall-seek so we can rehome once when the hall trips, before PID takes over
   * again.
   */
  private boolean wasSeekingRetractHall = false;

  private boolean wasSeekingDeployHall = false;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    deployMotor =
        new TalonFX(IntakeConstants.INTAKE_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredIntakeSpeed = 0;
    retractedLimitSwitch = new DigitalInput(IntakeConstants.INTAKE_HALL_EFFECT_PORT);
    deployedLimitSwitch = new DigitalInput(IntakeConstants.DEPLOYED_HALL_EFFECT_PORT);

    intakeState = IntakeDeployState.RETRACTED_STOPPED;

    intakeTalonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    deployTalonFXConfigs = new TalonFXConfiguration();

    deployTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    slot0Configs = deployTalonFXConfigs.Slot0;

    slot0Configs.kG = intakeKg.get(); // 0.2128 vaguely works
    slot0Configs.kS = intakeKs.get(); // 0.25 vaguely works
    slot0Configs.kV = intakeKv.get(); // 0.16 vaguely works
    slot0Configs.kA = intakeKa.get(); // 0.01 vaguely works

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = intakeKp.get(); // 1 vaguely works
    slot0Configs.kI = intakeKi.get(); // no output for integrated error
    slot0Configs.kD = intakeKd.get(); // 0.1 vaguely works

    // MOTION MAGIC EXPO
    motionMagicConfigs = deployTalonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // 0 gives us unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV =
        intakeExpoKv.get(); // was 0.16 kV is around 0.12 V/rps, might be 0.12-0.2
    motionMagicConfigs.MotionMagicExpo_kA =
        intakeExpoKa
            .get(); // was 0.1 Use a slower kA of 0.1 V/(rps/s) - the larger the kA, the smoother
    // and slower

    intakeCurrentLimitConfigs = intakeTalonFXConfigs.CurrentLimits;
    intakeCurrentLimitConfigs.StatorCurrentLimit = 25;
    intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    deployCurrentLimitConfigs = deployTalonFXConfigs.CurrentLimits;
    deployCurrentLimitConfigs.StatorCurrentLimit = 25;
    deployCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    intakeMotor.getConfigurator().apply(intakeTalonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);

    intakeMotor.set(0);

    isDeployed =
        () -> {
          return false;
        };
  }

  @Override
  public void periodic() {
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    updateSlot0Configs();
    updateMotionMagicConfigs();
    updateCurrentLimitConfigs();

    shouldStop();
    if (intakeState.equals(IntakeDeployState.RETRACTED_STOPPED)
        || intakeState.equals(IntakeDeployState.DEPLOYED_STOPPED)) {
      setDeploySpeed(0);
    } else if (!intakeState.equals(IntakeDeployState.SYSID)) {
      if (useDeployPositionControl) { // TODO when should we set this to true?
        setDeploySpeed(desiredDeploySpeed);
      } else {
        applyDeployPositionControl();
      }
    }
    home();
    intakeLogs();
  }

  private void shouldStop() {
    // we only trust that the intake is in the deploy position deadband if we're actually using
    // position control
    boolean atPositionDeadband =
        useDeployPositionControl
            && getDesiredAngle().getDegrees() - getCurrentAngle().getDegrees()
                <= IntakeConstants.POSITION_DEADBAND_DEGREES;
    if (intakeState.equals(IntakeDeployState.DEPLOYING)) {
      if (isDeployedLimitSwitchTriggered() || atPositionDeadband) {
        intakeState = IntakeDeployState.DEPLOYED_STOPPED;
      }
    } else if (intakeState.equals(IntakeDeployState.RETRACTING)) {
      if (isRetractedLimitSwitchTriggered() || atPositionDeadband) {
        intakeState = IntakeDeployState.RETRACTED_STOPPED;
      }
    }
  }

  private void applyDeployPositionControl() {
    deployMotor.setControl(m_request.withPosition(degreesToRevs(getDesiredAngle().getDegrees())));
    if (isDeployed.getAsBoolean()
        && deployCurrentLimitConfigs.StatorCurrentLimit != 25
        && getCurrentAngle().getDegrees() > 30) {
      deployCurrentLimitConfigs.StatorCurrentLimit = 25;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
    if ((!isDeployed.getAsBoolean() || getCurrentAngle().getDegrees() < 30)
        && deployCurrentLimitConfigs.StatorCurrentLimit != 60) {
      deployCurrentLimitConfigs.StatorCurrentLimit = 60;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
  }

  private void home() {
    if (isRetractedLimitSwitchTriggered()) {
      zeroIntakeDeploy(true);
    } else if (isDeployedLimitSwitchTriggered()) {
      zeroIntakeDeploy(false);
    }
  }

  public void setDesiredDeployPosition(boolean isRetracted) {
    if (isRetracted) {
      intakeState = IntakeDeployState.RETRACTING;
    } else {
      intakeState = IntakeDeployState.DEPLOYING;
    }
    useDeployPositionControl = true;
  }

  /** Returns the stored desired angle (always the true target, independent of deadband). */
  public Rotation2d getDesiredAngle() {
    Rotation2d desiredAngle;
    if (intakeState.equals(IntakeDeployState.RETRACTED_STOPPED)
        || intakeState.equals(IntakeDeployState.RETRACTING)) {
      desiredAngle = IntakeConstants.RETRACTED_POSITION;
    } else if (intakeState.equals(IntakeDeployState.DEPLOYED_STOPPED)
        || intakeState.equals(IntakeDeployState.DEPLOYING)) {
      desiredAngle = IntakeConstants.EXTENDED_POSITION;
    } else { // if it's not an expected state, let's just set it to be retracted automatically
      desiredAngle = IntakeConstants.RETRACTED_POSITION;
    }
    return desiredAngle;
  }

  public void setDeploySpeed(double speed) {
    useDeployPositionControl = false;

    // based on the direction of the speed, we can determine whether we're extending or retracting
    if (speed > 0) {
      intakeState = IntakeDeployState.DEPLOYING; // TODO see if this is possibly flipped irl
    } else if (speed < 0) {
      intakeState = IntakeDeployState.RETRACTING;
    }

    desiredDeploySpeed = speed;
    deployMotor.set(speed); // TODO redundant but we can keep it for safety
  }

  public void setIntakeSpeed(double speed) {
    desiredIntakeSpeed = speed;
    intakeMotor.set(desiredIntakeSpeed);
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = deployMotor.getPosition().getValueAsDouble();
    double deployAngleDegrees =
        motorPositionRevs
            / IntakeConstants.DEPLOY_GEARBOX_RATIO
            / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
            / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
            * 360.0
            % 360;
    return new Rotation2d(Math.toRadians(deployAngleDegrees));
  }

  public double degreesToRevs(double deployAngleDegrees) {
    return deployAngleDegrees
        / 360.0
        * IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        * IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        * IntakeConstants.DEPLOY_GEARBOX_RATIO;
  }

  public void zeroIntakeDeploy(boolean isRetracted) {
    if (isRetracted) {
      deployMotor.setPosition(degreesToRevs(IntakeConstants.RETRACTED_POSITION.getDegrees()));

    } else {
      deployMotor.setPosition(degreesToRevs(IntakeConstants.EXTENDED_POSITION.getDegrees()));
    }
  }

  public boolean isRetractedLimitSwitchTriggered() {
    return !retractedLimitSwitch.get();
  }

  public boolean isDeployedLimitSwitchTriggered() {
    return !deployedLimitSwitch.get();
  }

  public boolean isDeployed() {
    return intakeState.equals(IntakeDeployState.DEPLOYED_STOPPED)
        || intakeState.equals(IntakeDeployState.DEPLOYING);
  }

  private double getVelocityRadPerSec() {
    double motorRPS = deployMotor.getVelocity().getValueAsDouble();
    return motorRPS
        / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        / IntakeConstants.DEPLOY_GEARBOX_RATIO
        * 2
        * Math.PI;
  }

  private SysIdRoutine sysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(2),
            // this is the maximum voltage for the test
            Volts.of(18),
            // this is the duration of the test.
            // Note we use `until` when we return the command to abort if we hit intake deployed or
            // retracted positions
            Seconds.of(10),
            (state) -> Logger.recordOutput("Mech/Intake/SysID/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage; we log voltage/position/velocity ourselves in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> deployMotor.setVoltage(voltage.in(Volts)),
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "intake");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    boolean isSysIdOutOfBounds =
        angleDeg >= IntakeConstants.EXTENDED_ANGLE_DEGREES + SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg <= IntakeConstants.RETRACTED_ANGLE_DEGREES - SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Intake/SysID/Outofbounds", isSysIdOutOfBounds);

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
        .withName("Intake SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .dynamic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Intake SysId Dynamic " + direction);
  }

  public void updateSlot0Configs() {
    double newKp = intakeKp.get();
    double newKi = intakeKi.get();
    double newKd = intakeKd.get();
    double newKg = intakeKg.get();
    double newKs = intakeKs.get();
    double newKa = intakeKa.get();
    double newKv = intakeKv.get();

    if (newKp != slot0Configs.kP
        || newKi != slot0Configs.kI
        || newKd != slot0Configs.kD
        || newKa != slot0Configs.kA
        || newKv != slot0Configs.kV
        || newKg != slot0Configs.kG
        || newKs != slot0Configs.kS) {

      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      slot0Configs.kG = newKg;
      slot0Configs.kS = newKs;
      slot0Configs.kA = newKa;
      slot0Configs.kV = newKv;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
  }

  public void updateMotionMagicConfigs() {
    double newExpoKa = intakeExpoKa.get();
    double newExpoKv = intakeExpoKv.get();

    if (newExpoKa != motionMagicConfigs.MotionMagicExpo_kA
        || newExpoKv != motionMagicConfigs.MotionMagicExpo_kV) {

      motionMagicConfigs.MotionMagicExpo_kA = newExpoKa;
      motionMagicConfigs.MotionMagicExpo_kV = newExpoKv;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
  }

  public void updateCurrentLimitConfigs() {
    double newIntakeCurrentLimit = intakeCurrentLimit.get();

    if (newIntakeCurrentLimit != intakeCurrentLimitConfigs.StatorCurrentLimit) {
      intakeCurrentLimitConfigs.StatorCurrentLimit = newIntakeCurrentLimit;
      intakeMotor.getConfigurator().apply(intakeTalonFXConfigs);
    }
  }

  public void intakeLogs() {
    TalonFXLogger.log(deployMotor, "Mech", "Intake", "Deploy");
    TalonFXLogger.log(intakeMotor, "Mech", "Intake", "Intake");

    Logger.recordOutput("Mech/Intake/Deploy/Current Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Deploy/Desired Angle", getDesiredAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Deploy/Desired Speed", desiredDeploySpeed);
    Logger.recordOutput(
        "Mech/Intake/Deploy/Current Limit", deployCurrentLimitConfigs.StatorCurrentLimit);

    Logger.recordOutput("Mech/Intake/Retracted Limit Switch", isRetractedLimitSwitchTriggered());
    Logger.recordOutput("Mech/Intake/Deployed Limit Switch", isDeployedLimitSwitchTriggered());
    Logger.recordOutput("Mech/Intake/Hall Assist Active", hallAssistActive);
    Logger.recordOutput("Mech/Intake/IsDeployed", isDeployed.getAsBoolean());
    Logger.recordOutput("Mech/Intake/Intake/Desired Intake Speed", desiredIntakeSpeed);
    Logger.recordOutput(
        "Mech/Intake/Intake/Current Limit", intakeCurrentLimitConfigs.StatorCurrentLimit);

    // SysID
    Logger.recordOutput(
        "Mech/Intake/SysID/intakeSysIDRunning", intakeState.equals(IntakeDeployState.SYSID));
    if (intakeState.equals(IntakeDeployState.SYSID)) {
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVoltage", deployMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Intake/SysID/intakePosition",
          getCurrentAngle().getRadians() / (2.0 * Math.PI)); // rotations
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVelocity", getVelocityRadPerSec() / (2.0 * Math.PI)); // rot/s
    }
  }
}
