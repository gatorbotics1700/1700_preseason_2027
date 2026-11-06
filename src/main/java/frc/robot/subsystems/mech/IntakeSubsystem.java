package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.Calculations;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// TODO: the LoggedNetworkNumbers (dashboard tuning numbers) below are still static, which breaks
// the "no static" rule in the mechanism checklist. (The static motor settings moved into
// IntakeIOTalonFX as normal fields in Step 4.)
public class IntakeSubsystem extends SubsystemBase {

  // The hardware layer: rollers, deploy arm and limit switches all go through it.
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Dashboard warnings. Not private, so the tests in this folder can check them.
  final Alert leftRollerDisconnectedAlert;
  final Alert rightRollerDisconnectedAlert;
  final Alert leftDeployDisconnectedAlert;
  final Alert rightDeployDisconnectedAlert;
  final Alert armNotArrivingAlert =
      new Alert("Intake: arm was told to move but has not arrived.", AlertType.kWarning);

  // The arm should finish a move well within this time. TUNED, NOT DERIVED (2026-11-06)
  private static final double MAX_MOVE_TIME_SECONDS = 2.0;
  private static final double LOOP_TIME_SECONDS = 0.02; // periodic() runs every 20 ms
  // We count loops instead of reading a clock, so a replayed log gives the same answer.
  private static final int MAX_MOVE_LOOPS = (int) (MAX_MOVE_TIME_SECONDS / LOOP_TIME_SECONDS);
  private int loopsSpentMoving = 0;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 3;

  public enum IntakeDeployState {
    SYSID,
    DEPLOYING,
    RETRACTING,
    DEPLOYED_STOPPED,
    RETRACTED_STOPPED
  }

  private IntakeDeployState intakeState;

  // TODO (2026-11-02), CHECK WITH TEAM AT CODE REVIEW: these defaults disagree with the
  // "vaguely works" notes in the constructor (kG 0.2128 vs 0, kS 0.25 vs 0, kP 1 vs 6). Which
  // numbers did the robot actually run with? The robot goes back to these defaults on every
  // restart, so the tested values belong here, marked TUNED, NOT DERIVED (date).
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

  // See the 70 A vs 90 A note in IntakeConstants.
  public static final LoggedNetworkNumber intakeCurrentLimit =
      new LoggedNetworkNumber(
          "/Tuning/Intake/Intake Current Limit",
          IntakeConstants.ROLLER_DASHBOARD_CURRENT_LIMIT_AMPS);

  private boolean useDeployPositionControl = false;
  private double desiredIntakeSpeed;
  private double desiredDeploySpeed;

  // private BooleanSupplier isDeployed;

  /** Builds the intake with the real hardware. RobotContainer uses this one. */
  public IntakeSubsystem() {
    this(new IntakeIOTalonFX());
  }

  /** Builds the intake with any hardware layer: real, simulated, or one that does nothing. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    desiredIntakeSpeed = 0;
    intakeState = IntakeDeployState.RETRACTED_STOPPED;

    leftRollerDisconnectedAlert =
        makeDisconnectedAlert("left roller", IntakeConstants.LEFT_INTAKE_MOTOR_CAN_ID);
    rightRollerDisconnectedAlert =
        makeDisconnectedAlert("right roller", IntakeConstants.RIGHT_INTAKE_MOTOR_CAN_ID);
    leftDeployDisconnectedAlert =
        makeDisconnectedAlert("left deploy", IntakeConstants.INTAKE_LEFT_DEPLOY_MOTOR_CAN_ID);
    rightDeployDisconnectedAlert =
        makeDisconnectedAlert("right deploy", IntakeConstants.INTAKE_RIGHT_DEPLOY_MOTOR_CAN_ID);

    // Send the starting gains from the dashboard numbers. Older notes on values that "vaguely
    // work": kG 0.2128, kS 0.25, kV 0.16, kA 0.01, kP 1, kI 0 (no output for integrated error),
    // kD 0.1; Expo kV was 0.16 (around 0.12 V/rps, might be 0.12-0.2), Expo kA was 0.1 (the
    // larger the kA, the smoother and slower).
    updateSlot0Configs();
    updateMotionMagicConfigs();

    // isDeployed =
    //     () -> {
    //       return false;
    //     };
  }

  @Override
  public void periodic() {
    // Read the hardware first, so every decision below uses this loop's values.
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Intake/IO", inputs);
    updateAlerts();

    // Update PID gains from NetworkTables if they've changed, and reapply configs
    updateSlot0Configs();
    updateMotionMagicConfigs();
    updateCurrentLimitConfigs();

    shouldStop();
    updateArmNotArrivingAlert();
    if (intakeState.equals(IntakeDeployState.RETRACTED_STOPPED)
        || intakeState.equals(IntakeDeployState.DEPLOYED_STOPPED)) {
      setDeploySpeed(0);
    } else if (!intakeState.equals(IntakeDeployState.SYSID)) {
      if (useDeployPositionControl) {
        applyDeployPositionControl();
      } else {
        setDeploySpeed(desiredDeploySpeed);
      }
    }
    // TODO - Is home as defined computationally expensive?
    home();
    intakeLogs();
  }

  /** Makes a dashboard error saying which motor is disconnected, and its CAN ID. */
  private Alert makeDisconnectedAlert(String motorName, int canId) {
    return new Alert(
        "Intake: " + motorName + " motor disconnected (CAN " + canId + ").", AlertType.kError);
  }

  /** Turns each motor's alert on while that motor is disconnected, and off when it is back. */
  private void updateAlerts() {
    leftRollerDisconnectedAlert.set(!inputs.leftRollerConnected);
    rightRollerDisconnectedAlert.set(!inputs.rightRollerConnected);
    leftDeployDisconnectedAlert.set(!inputs.leftDeployConnected);
    rightDeployDisconnectedAlert.set(!inputs.rightDeployConnected);
  }

  /**
   * Counts how many loops the arm has been moving, and warns if a move takes too long. A stuck,
   * unpowered or jammed arm never reaches its target or a switch, so it stays "moving".
   */
  private void updateArmNotArrivingAlert() {
    boolean isMoving =
        intakeState.equals(IntakeDeployState.DEPLOYING)
            || intakeState.equals(IntakeDeployState.RETRACTING);
    if (isMoving) {
      loopsSpentMoving = loopsSpentMoving + 1;
    } else {
      loopsSpentMoving = 0;
    }
    armNotArrivingAlert.set(loopsSpentMoving > MAX_MOVE_LOOPS);
  }

  private void shouldStop() {
    // we only trust that the intake is in the deploy position deadband if we're actually using
    // position control
    boolean atPositionDeadband =
        useDeployPositionControl
            && Calculations.withinDeadband(
                getDesiredAngle().getDegrees(),
                getCurrentAngle().getDegrees(),
                IntakeConstants.POSITION_DEADBAND_DEGREES,
                true);
    boolean currentLimitReached =
        inputs.leftDeployCurrentAmps >= IntakeConstants.DEPLOY_JAM_CURRENT_AMPS
            || inputs.rightDeployCurrentAmps >= IntakeConstants.DEPLOY_JAM_CURRENT_AMPS;
    if (intakeState.equals(IntakeDeployState.DEPLOYING)) {
      if (isDeployedLimitSwitchTriggered() || atPositionDeadband || currentLimitReached) {
        intakeState = IntakeDeployState.DEPLOYED_STOPPED;
      }
    } else if (intakeState.equals(IntakeDeployState.RETRACTING)) {
      if (isRetractedLimitSwitchTriggered() || atPositionDeadband || currentLimitReached) {
        intakeState = IntakeDeployState.RETRACTED_STOPPED;
      }
    }
  }

  private void applyDeployPositionControl() {
    io.setDeployTargetRotations(degreesToRevs(getDesiredAngle().getDegrees()));
    // Old experiment, turned off. leftDeployMotor and deployTalonFXConfigs now live in
    // IntakeIOTalonFX, so this would need an IntakeIO method to work again.
    // if (isDeployed.getAsBoolean()
    //     && deployCurrentLimitConfigs.StatorCurrentLimit != 25
    //     && getCurrentAngle().getDegrees() > 30) {
    //   deployCurrentLimitConfigs.StatorCurrentLimit = 25;
    //   leftDeployMotor.getConfigurator().apply(deployTalonFXConfigs);
    // }
    // if ((!isDeployed.getAsBoolean() || getCurrentAngle().getDegrees() < 30)
    //     && deployCurrentLimitConfigs.StatorCurrentLimit != 60) {
    //   deployCurrentLimitConfigs.StatorCurrentLimit = 60;
    //   leftDeployMotor.getConfigurator().apply(deployTalonFXConfigs);
    // }
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
    io.setDeploySpeed(speed); // TODO redundant but we can keep it for safety
  }

  public void setIntakeSpeed(double speed) {
    desiredIntakeSpeed = speed;
    io.setRollerSpeed(desiredIntakeSpeed);
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = inputs.leftDeployPositionRotations;
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
      io.resetDeployPositionRotations(
          degreesToRevs(IntakeConstants.RETRACTED_POSITION.getDegrees()));
    } else {
      io.resetDeployPositionRotations(
          degreesToRevs(IntakeConstants.EXTENDED_POSITION.getDegrees()));
    }
  }

  public boolean isRetractedLimitSwitchTriggered() {
    return inputs.retractedSwitchPressed;
  }

  public boolean isDeployedLimitSwitchTriggered() {
    return inputs.deployedSwitchPressed;
  }

  public boolean isDeployed() {
    return intakeState.equals(IntakeDeployState.DEPLOYED_STOPPED)
        || intakeState.equals(IntakeDeployState.DEPLOYING);
  }

  private double getVelocityRadPerSec() {
    double motorRPS = inputs.leftDeployVelocityRps;
    return motorRPS
        / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        / IntakeConstants.DEPLOY_GEARBOX_RATIO
        * 2
        * Math.PI;
  }

  /**
   * Called by SysId commands to indicate test is running; we log voltage/position/velocity in
   * periodic().
   */
  private void setSysIdRunning(boolean running) {
    if (running) {
      intakeState = IntakeDeployState.SYSID;
    } else {
      intakeState =
          IntakeDeployState.RETRACTED_STOPPED; // TODO possibly change but it doesn't really matter
    }
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
            (voltage) -> io.setDeployVoltage(voltage.in(Volts)),
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

  /** Sends the dashboard's deploy gains to the hardware. The hardware ignores unchanged ones. */
  public void updateSlot0Configs() {
    io.setDeployPidGains(intakeKp.get(), intakeKi.get(), intakeKd.get());
    io.setDeployFeedforwardGains(intakeKg.get(), intakeKs.get(), intakeKv.get(), intakeKa.get());
  }

  /** Sends the dashboard's Motion Magic Expo gains to the hardware. */
  public void updateMotionMagicConfigs() {
    io.setDeployMotionMagicExpoGains(intakeExpoKv.get(), intakeExpoKa.get());
  }

  public void updateCurrentLimitConfigs() {
    io.setRollerCurrentLimitAmps(intakeCurrentLimit.get());
  }

  public void intakeLogs() {
    Logger.recordOutput("Mech/Intake/Deploy/Current Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Deploy/Desired Angle", getDesiredAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Deploy/Desired Speed", desiredDeploySpeed);
    Logger.recordOutput("Mech/Intake/Deploy/Current Limit", inputs.deployCurrentLimitAmps);

    Logger.recordOutput("Mech/Intake/Retracted Limit Switch", isRetractedLimitSwitchTriggered());
    Logger.recordOutput("Mech/Intake/Deployed Limit Switch", isDeployedLimitSwitchTriggered());
    Logger.recordOutput("Mech/Intake/IsDeployed", isDeployed());
    Logger.recordOutput("Mech/Intake/Intake/Desired Intake Speed", desiredIntakeSpeed);
    Logger.recordOutput("Mech/Intake/Intake/Current Limit", inputs.leftRollerCurrentLimitAmps);
    Logger.recordOutput("Mech/Intake/intake deploy state", intakeState.toString());
    // SysID
    Logger.recordOutput(
        "Mech/Intake/SysID/intakeSysIDRunning", intakeState.equals(IntakeDeployState.SYSID));
    if (intakeState.equals(IntakeDeployState.SYSID)) {
      Logger.recordOutput("Mech/Intake/SysID/intakeVoltage", inputs.leftDeployVoltageVolts);
      Logger.recordOutput(
          "Mech/Intake/SysID/intakePosition",
          getCurrentAngle().getRadians() / (2.0 * Math.PI)); // rotations
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVelocity", getVelocityRadPerSec() / (2.0 * Math.PI)); // rot/s
    }
  }
}
