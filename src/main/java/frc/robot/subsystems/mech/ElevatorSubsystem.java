package frc.robot.subsystems.mech;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoopTimer;
import frc.robot.util.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * The decisions for a two-stage cascade elevator, talking to hardware only through ElevatorIO. It
 * won't move to a goal until it has seen the bottom limit switch (homed).
 */
public class ElevatorSubsystem extends SubsystemBase {

  /** What the elevator is doing: nothing, homing, moving to a height, or driving by voltage. */
  public enum ControlMode {
    IDLE,
    HOMING,
    POSITION,
    VOLTAGE
  }

  // A bit more than Motion Magic can stop short by, so it always switches to the gentle push.
  private static final double LANDING_SLACK_METERS = 0.005;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ControlMode mode = ControlMode.IDLE;
  private double goalMeters = ElevatorConstants.STOW_HEIGHT_METERS;
  private double voltageOutputVolts = 0.0;
  private boolean homed = false;
  private boolean limitWasPressed = false;
  private boolean homingFailed = false;
  private final LoopTimer homingTimer = new LoopTimer(ElevatorConstants.HOMING_TIMEOUT_LOOPS);
  private final LoopTimer notArrivedTimer =
      new LoopTimer(ElevatorConstants.NOT_ARRIVED_TIMEOUT_LOOPS);

  // Not private, so tests can check them.
  final Alert disconnectedAlert =
      new Alert(
          "Elevator: motor (CAN " + ElevatorConstants.MOTOR_CAN_ID + ") disconnected.",
          AlertType.kError);
  final Alert homingFailedAlert =
      new Alert(
          "Elevator: homing gave up, bottom switch never pressed. Check the switch.",
          AlertType.kError);
  final Alert notArrivedAlert =
      new Alert("Elevator: told to move but not at the goal after 2 s.", AlertType.kWarning);

  // Tuning gains on the dashboard, under /Tuning/Elevator. Not private, so tests can set them.
  final TunableGains gains =
      new TunableGains(
          "Elevator",
          ElevatorConstants.KP,
          ElevatorConstants.KI,
          ElevatorConstants.KD,
          ElevatorConstants.KS,
          ElevatorConstants.KG,
          ElevatorConstants.KV,
          ElevatorConstants.KA);

  /** Builds the elevator with the real hardware. */
  public ElevatorSubsystem() {
    this(new ElevatorIOTalonFX());
  }

  /** Builds the elevator with any hardware layer: real, simulated, or one that does nothing. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  /** Runs every 20 ms: read the hardware, check for problems, decide, then log. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Elevator/IO", inputs);

    disconnectedAlert.set(!inputs.motorConnected);
    homeOnSwitchPress();
    sendGainsIfChanged();
    runMode();
    updateNotArrivedAlert();
    logOutputs();
  }

  /** Does what the current mode asks. Disabling stops it, so an old goal never resumes. */
  private void runMode() {
    if (DriverStation.isDisabled()) {
      mode = ControlMode.IDLE;
    }
    if (mode == ControlMode.HOMING) {
      runHoming();
    } else if (mode == ControlMode.POSITION) {
      runPositionControl();
    } else if (mode == ControlMode.VOLTAGE) {
      runVoltageControl();
    } else {
      io.stop();
    }
  }

  /** Drives down slowly until the bottom switch closes, or gives up after the timeout. */
  private void runHoming() {
    boolean timedOut = homingTimer.update(true);
    if (inputs.bottomLimitPressed) {
      // homeOnSwitchPress() already zeroed the position.
      setGoal(ElevatorConstants.STOW_HEIGHT_METERS);
      io.stop();
    } else if (timedOut) {
      homingFailed = true;
      homingFailedAlert.set(true);
      mode = ControlMode.IDLE;
      io.stop();
    } else {
      io.setVoltage(ElevatorConstants.HOMING_VOLTS);
    }
  }

  /** Warns when a goal has not been reached after the timeout. A new goal starts the count. */
  private void updateNotArrivedAlert() {
    boolean movingToGoal = mode == ControlMode.POSITION && !atGoal();
    notArrivedAlert.set(notArrivedTimer.update(movingToGoal));
  }

  /** Sets the position to 0 when the bottom switch closes: the only true height reference. */
  private void homeOnSwitchPress() {
    // Only on the press itself, not every loop it stays pressed.
    if (inputs.bottomLimitPressed && !limitWasPressed) {
      io.resetPosition(ElevatorConstants.MIN_HEIGHT_METERS);
      homed = true;
    }
    limitWasPressed = inputs.bottomLimitPressed;
  }

  /** Sends the dashboard gains to the motor, only when one of them changed. */
  private void sendGainsIfChanged() {
    if (gains.changed()) {
      io.setGains(
          gains.kP.get(),
          gains.kI.get(),
          gains.kD.get(),
          gains.kS.get(),
          gains.kG.get(),
          gains.kV.get(),
          gains.kA.get());
    }
  }

  /** Moves to the goal height, or sets the carriage down gently when the goal is the bottom. */
  private void runPositionControl() {
    boolean goalIsBottom =
        goalMeters
            <= ElevatorConstants.MIN_HEIGHT_METERS + ElevatorConstants.POSITION_TOLERANCE_METERS;
    boolean inLandingZone =
        inputs.positionMeters <= ElevatorConstants.LANDING_HEIGHT_METERS + LANDING_SLACK_METERS;

    if (!homed) {
      // The height is unknown, so any move could hit a hard stop.
      io.stop();
    } else if (goalIsBottom && inputs.bottomLimitPressed) {
      // Resting on the hard stop: no power, so no holding current.
      io.stop();
    } else if (goalIsBottom && inLandingZone) {
      // Last few cm: a gentle push instead of driving the profile into the stop.
      io.setVoltage(ElevatorConstants.LANDING_VOLTS);
    } else if (goalIsBottom) {
      io.setPositionSetpoint(ElevatorConstants.LANDING_HEIGHT_METERS);
    } else {
      io.setPositionSetpoint(goalMeters);
    }
  }

  /** Drives by voltage, but never down into a pressed bottom switch. */
  private void runVoltageControl() {
    boolean drivingIntoBottom = voltageOutputVolts < 0 && inputs.bottomLimitPressed;
    if (drivingIntoBottom) {
      io.stop();
    } else {
      io.setVoltage(voltageOutputVolts);
    }
  }

  /** Logs what the elevator was told to do. What it did is already logged in inputs. */
  private void logOutputs() {
    double heightMeters = getPositionMeters();
    Logger.recordOutput("Mech/Elevator/GoalMeters", goalMeters);
    Logger.recordOutput("Mech/Elevator/ControlMode", mode.name());
    Logger.recordOutput("Mech/Elevator/Homed", homed);
    Logger.recordOutput("Mech/Elevator/AtGoal", atGoal());
    Logger.recordOutput("Mech/Elevator/ComponentPoses", componentPoses(heightMeters));
    Logger.recordOutput("Mech/Elevator/ComponentPosesGoal", componentPoses(goalMeters));
  }

  /** 3D model poses for a carriage height: stage 1 rises 1 / RIGGING_RATIO as far. */
  public static Pose3d[] componentPoses(double carriageHeightMeters) {
    double stage1HeightMeters = carriageHeightMeters / ElevatorConstants.RIGGING_RATIO;
    Pose3d stage1 = new Pose3d(0, 0, stage1HeightMeters, new Rotation3d());
    Pose3d carriage = new Pose3d(0, 0, carriageHeightMeters, new Rotation3d());
    return new Pose3d[] {stage1, carriage};
  }

  /** Limits a height to the safe travel range. */
  public static double clampHeight(double heightMeters) {
    return MathUtil.clamp(
        heightMeters, ElevatorConstants.MIN_HEIGHT_METERS, ElevatorConstants.MAX_HEIGHT_METERS);
  }

  /** Starts homing: drive down slowly until the bottom switch closes. */
  public void startHoming() {
    mode = ControlMode.HOMING;
    homingTimer.reset();
    homingFailed = false;
    homingFailedAlert.set(false);
  }

  /** Moves to a height and holds it there. Needs homing first. */
  public void setGoal(double heightMeters) {
    mode = ControlMode.POSITION;
    goalMeters = clampHeight(heightMeters);
    notArrivedTimer.reset();
  }

  /** Drives by voltage. Negative is down. It stops itself on the bottom switch. */
  public void setVoltage(double volts) {
    mode = ControlMode.VOLTAGE;
    voltageOutputVolts = volts;
  }

  /** Stops driving the motor. */
  public void stop() {
    mode = ControlMode.IDLE;
  }

  /** The carriage height now. */
  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  /** The height it was last told to go to. */
  public double getGoalMeters() {
    return goalMeters;
  }

  /** What the elevator is doing now. */
  public ControlMode getControlMode() {
    return mode;
  }

  /** True once the bottom switch has been seen, so the height is known. */
  public boolean isHomed() {
    return homed;
  }

  /** True while homing is still running. */
  public boolean isHoming() {
    return mode == ControlMode.HOMING;
  }

  /** True when the last homing gave up without seeing the switch. */
  public boolean homingFailed() {
    return homingFailed;
  }

  /** True while the bottom limit switch is pressed. */
  public boolean isBottomLimitPressed() {
    return inputs.bottomLimitPressed;
  }

  /** True when homed, holding a height, and within tolerance of the goal. */
  public boolean atGoal() {
    double errorMeters = Math.abs(inputs.positionMeters - goalMeters);
    return homed
        && mode == ControlMode.POSITION
        && errorMeters <= ElevatorConstants.POSITION_TOLERANCE_METERS;
  }
}
