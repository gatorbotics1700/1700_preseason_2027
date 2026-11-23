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

// TODO NOTE FROM REFACTOR - the dashboard tuning numbers below are "public static", which makes
// them global variables: any file in the robot code could read or change them, so a wrong value
// would be hard to trace back. They also get created when Java first loads this class, not when
// the robot builds the intake. They belong to this intake, so they should be normal private
// fields. No other file uses them today, and the robot has only one intake, so this causes no bug
// yet.
/**
 * The intake's decisions: when to run the rollers, and when to move, stop and re-zero the deploy
 * arm. All hardware goes through an IntakeIO, so tests and the simulator can stand in for it.
 */
public class IntakeSubsystem extends SubsystemBase {

  // The hardware layer: rollers, deploy arm and limit switches all go through it.
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Dashboard warnings. Left without "private" on purpose, so the intake tests can check them.
  final Alert leftRollerDisconnectedAlert;
  final Alert rightRollerDisconnectedAlert;
  final Alert leftDeployDisconnectedAlert;
  final Alert rightDeployDisconnectedAlert;
  final Alert armNotArrivingAlert =
      new Alert("Intake: arm was told to move but has not arrived.", AlertType.kWarning);

  // Assume that the deploy or retract takes a max of 2 secs. If it takes longer, then warning is
  // displayed.
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

  // TODO NOTE FROM REFACTOR - 
  // Expo kV and Expo kA below also look swapped compared with those notes. Which numbers did the
  // robot actually run with? The robot goes back to these defaults on every restart, so the
  // tested values belong here.
  // Tuning hints from the same notes: kI 0 means no push from built-up error. Expo kV is around
  // 0.12 V/rps (maybe 0.12-0.2). A larger Expo kA makes the move smoother and slower.
  // PID gains for the deploy arm, tunable on the dashboard
  private static final LoggedNetworkNumber intakeKp =
      new LoggedNetworkNumber("/Tuning/Intake/kP", 6);
  private static final LoggedNetworkNumber intakeKi =
      new LoggedNetworkNumber("/Tuning/Intake/kI", 0.0);
  private static final LoggedNetworkNumber intakeKd =
      new LoggedNetworkNumber("/Tuning/Intake/kD", 0.1);

  // Feedforward gains for the deploy arm, tunable on the dashboard
  private static final LoggedNetworkNumber intakeKg =
      new LoggedNetworkNumber("/Tuning/Intake/kG", 0.0);
  private static final LoggedNetworkNumber intakeKs =
      new LoggedNetworkNumber("/Tuning/Intake/kS", 0.0);
  private static final LoggedNetworkNumber intakeKv =
      new LoggedNetworkNumber("/Tuning/Intake/kV", 0.16);
  private static final LoggedNetworkNumber intakeKa =
      new LoggedNetworkNumber("/Tuning/Intake/kA", 0.01);

  private static final LoggedNetworkNumber intakeExpoKa =
      new LoggedNetworkNumber("/Tuning/Intake/Expo kA", 0.16);
  private static final LoggedNetworkNumber intakeExpoKv =
      new LoggedNetworkNumber("/Tuning/Intake/Expo kV", 0.1);

  private static final LoggedNetworkNumber intakeCurrentLimit =
      new LoggedNetworkNumber(
          "/Tuning/Intake/Intake Current Limit",
          IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS);

  private boolean useDeployPositionControl = false;
  private double desiredIntakeSpeed;
  private double desiredDeploySpeed;

  /**
   * Builds the intake with any hardware layer: real, simulated, or one that does nothing.
   * RobotContainer picks which one.
   */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    desiredIntakeSpeed = 0;
    // TODO NOTE FROM REFACTOR - the code assumes the arm starts retracted. If the robot turns on
    // with the arm out, the angle is wrong until the arm presses a limit switch.
    intakeState = IntakeDeployState.RETRACTED_STOPPED;

    leftRollerDisconnectedAlert =
        makeDisconnectedAlert("left roller", IntakeConstants.LEFT_INTAKE_MOTOR_CAN_ID);
    rightRollerDisconnectedAlert =
        makeDisconnectedAlert("right roller", IntakeConstants.RIGHT_INTAKE_MOTOR_CAN_ID);
    leftDeployDisconnectedAlert =
        makeDisconnectedAlert("left deploy", IntakeConstants.INTAKE_LEFT_DEPLOY_MOTOR_CAN_ID);
    rightDeployDisconnectedAlert =
        makeDisconnectedAlert("right deploy", IntakeConstants.INTAKE_RIGHT_DEPLOY_MOTOR_CAN_ID);

    // Send the starting gains from the dashboard numbers to the motors.
    updateSlot0Configs();
    updateMotionMagicConfigs();
  }

  @Override
  public void periodic() {
    // Read the hardware first, so every decision below uses this loop's values.
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Intake/IO", inputs);
    updateAlerts();

    // Send the dashboard's numbers to the motors. The IO layer only sends ones that changed.
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

  /** Stops a moving arm when it hits a limit switch, gets close to its target, or jams. */
  private void shouldStop() {
    // "Close to its target" only counts when the arm is being sent to a position, not when it is
    // being run at a speed.
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

  /** Sends the arm toward its target angle. */
  private void applyDeployPositionControl() {
    io.setDeployTargetRotations(degreesToRevs(getDesiredAngle().getDegrees()));
    // Old experiment, turned off: lower the arm's current limit once it is out. leftDeployMotor
    // and deployTalonFXConfigs now live in IntakeIOTalonFX, and isDeployed was never filled in,
    // so this would need an IntakeIO method to work again.
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

  // TODO NOTE FROM REFACTOR - this runs every loop. While a switch stays pressed, it resets both
  // deploy motors 50 times a second. Each reset waits for the motor to answer, which can slow the
  // robot's loop. Resetting once, when the switch first becomes pressed, would fix it.
  /** Resets the arm's angle when a limit switch is pressed, so small errors don't build up. */
  private void home() {
    if (isRetractedLimitSwitchTriggered()) {
      zeroIntakeDeploy(true);
    } else if (isDeployedLimitSwitchTriggered()) {
      zeroIntakeDeploy(false);
    }
  }

  /** Sends the arm in (true) or out (false). */
  public void setDesiredDeployPosition(boolean isRetracted) {
    if (isRetracted) {
      intakeState = IntakeDeployState.RETRACTING;
    } else {
      intakeState = IntakeDeployState.DEPLOYING;
    }
    useDeployPositionControl = true;
  }

  /** The angle the arm is headed for, or is resting at: retracted or deployed. */
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

  /** Runs the arm at a speed from -1 (retract) to 1 (deploy), instead of to a position. */
  public void setDeploySpeed(double speed) {
    useDeployPositionControl = false;

    // The sign of the speed says which way the arm is going.
    if (speed > 0) {
      intakeState = IntakeDeployState.DEPLOYING; // TODO see if this is possibly flipped irl
    } else if (speed < 0) {
      intakeState = IntakeDeployState.RETRACTING;
    }

    desiredDeploySpeed = speed;
    // periodic() sends this speed again every loop; sending it now too starts the arm sooner.
    io.setDeploySpeed(speed);
  }

  /** Runs the rollers at a speed from -1 to 1. */
  public void setIntakeSpeed(double speed) {
    desiredIntakeSpeed = speed;
    io.setRollerSpeed(desiredIntakeSpeed);
  }

  // TODO NOTE FROM REFACTOR - this uses the left deploy motor only; the right one is never
  // checked. The "% 360" does nothing useful, because the arm never turns a full circle.
  /** The arm's angle now: 0 = retracted, 95 = deployed. */
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

  /** Turns an arm angle into deploy motor turns (rotations), through the gears. */
  public double degreesToRevs(double deployAngleDegrees) {
    return deployAngleDegrees
        / 360.0
        * IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        * IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        * IntakeConstants.DEPLOY_GEARBOX_RATIO;
  }

  /** Tells the motors the arm is now fully in (true) or fully out (false). */
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

  /** True if the arm is out, or on its way out. */
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

  // TODO NOTE FROM REFACTOR - when a test ends, the state is always set to RETRACTED_STOPPED, even
  // if the arm is out.
  /**
   * Marks a SysId test as running or done. SysId is a WPILib tool that runs the arm at set voltages
   * and measures how it moves, to work out tuning gains.
   */
  private void setSysIdRunning(boolean running) {
    if (running) {
      intakeState = IntakeDeployState.SYSID;
    } else {
      intakeState = IntakeDeployState.RETRACTED_STOPPED;
    }
  }

  /** Sets up the SysId test: how fast the voltage rises, its top, and how long a test runs. */
  private SysIdRoutine sysIdRoutine() {
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // How fast the voltage rises in the slow test
            Volts.per(Second).of(2),
            // The voltage for the fast test.
            // TODO NOTE FROM REFACTOR - 18 V is more than the 12 V battery can give.
            Volts.of(18),
            // The longest a test can run. The commands below also stop it at either end of travel.
            Seconds.of(10),
            (state) -> Logger.recordOutput("Mech/Intake/SysID/SysIdState", state.toString()));

    // How SysId moves the arm. We log voltage, position and speed ourselves in intakeLogs().
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setDeployVoltage(voltage.in(Volts)),
            null, // null: we log it ourselves, so it goes in the same log file as everything else
            this,
            "intake");
    return new SysIdRoutine(config, mechanism);
  }

  // TODO NOTE FROM REFACTOR - the margin puts these stops at 98 and -3 degrees, outside the
  // travel, so a test can push the arm past its ends.
  /** True when a SysId test has gone past either end of the arm's travel, so it should stop. */
  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    boolean isSysIdOutOfBounds =
        angleDeg >= IntakeConstants.EXTENDED_ANGLE_DEGREES + SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg <= IntakeConstants.RETRACTED_ANGLE_DEGREES - SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Intake/SysID/Outofbounds", isSysIdOutOfBounds);

    return isSysIdOutOfBounds;
  }

  /** SysId's slow test: raises the voltage slowly, to see how speed follows voltage. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .quasistatic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Intake SysId Quasistatic " + direction);
  }

  /** SysId's fast test: jumps straight to a set voltage, to see how quickly the arm speeds up. */
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

  /** Sends the dashboard's roller current limit to the hardware. */
  public void updateCurrentLimitConfigs() {
    io.setRollerCurrentLimitAmps(intakeCurrentLimit.get());
  }

  /** Logs what the intake decided this loop, for AdvantageScope. */
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
