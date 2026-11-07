package frc.robot.subsystems.mech;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The decisions for a two-stage cascade elevator. It talks to the hardware only through ElevatorIO,
 * so tests can use ElevatorIOSim and the simulator can use ElevatorIOSimPhysics.
 *
 * <p>Every height is the carriage height in meters above the bottom hard stop. The motor's position
 * reads 0 at boot wherever the carriage really is, so the elevator refuses to move to a goal until
 * it has seen the bottom limit switch (homed). What it must do is written in
 * docs/elevator-requirements.md.
 *
 * <p>For AdvantageScope it logs, under Mech/Elevator/:
 *
 * <ul>
 *   <li>ComponentPoses: where each moving part of the 3D model is (0 = stage 1, 1 = carriage)
 *   <li>ComponentPosesGoal: the same for the goal, drawn as a see-through "ghost"
 *   <li>Mechanism2d: a 2D side view
 * </ul>
 */
public class ElevatorSubsystem extends SubsystemBase {

  /** What the elevator is doing: nothing, homing, moving to a height, or driving by voltage. */
  public enum ControlMode {
    IDLE,
    HOMING,
    POSITION,
    VOLTAGE
  }

  // Within this much of the landing height counts as "there". It is a little more than the motor
  // can stop short by, so the elevator always switches from Motion Magic to the gentle push.
  private static final double LANDING_SLACK_METERS = 0.005;

  // The hardware layer: real, simulated, or one that does nothing.
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ControlMode mode = ControlMode.IDLE;
  private double goalMeters = ElevatorConstants.STOW_HEIGHT_METERS;
  private double voltageOutputVolts = 0.0;
  private boolean homed = false;
  private boolean limitWasPressed = false;
  private boolean homingFailed = false;
  // Loops counted for the homing timeout and the "not arrived" alert (one loop = 20 ms).
  private int homingLoops = 0;
  private int notArrivedLoops = 0;

  // Dashboard warning. Not private, so the tests in this folder can check it.
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

  // Gains you can change on the dashboard while tuning, under /Tuning/Elevator.
  // Not private, so the tests in this folder can type on the pretend dashboard.
  final LoggedNetworkNumber kP =
      new LoggedNetworkNumber("/Tuning/Elevator/kP", ElevatorConstants.KP);
  final LoggedNetworkNumber kI =
      new LoggedNetworkNumber("/Tuning/Elevator/kI", ElevatorConstants.KI);
  final LoggedNetworkNumber kD =
      new LoggedNetworkNumber("/Tuning/Elevator/kD", ElevatorConstants.KD);
  final LoggedNetworkNumber kS =
      new LoggedNetworkNumber("/Tuning/Elevator/kS", ElevatorConstants.KS);
  final LoggedNetworkNumber kG =
      new LoggedNetworkNumber("/Tuning/Elevator/kG", ElevatorConstants.KG);
  final LoggedNetworkNumber kV =
      new LoggedNetworkNumber("/Tuning/Elevator/kV", ElevatorConstants.KV);
  final LoggedNetworkNumber kA =
      new LoggedNetworkNumber("/Tuning/Elevator/kA", ElevatorConstants.KA);

  // The gains the motor has now, so new ones are sent only when something changed.
  private final double[] sentGains = {
    ElevatorConstants.KP,
    ElevatorConstants.KI,
    ElevatorConstants.KD,
    ElevatorConstants.KS,
    ElevatorConstants.KG,
    ElevatorConstants.KV,
    ElevatorConstants.KA
  };

  // 2D side view: the fixed stage, and a carriage line that grows with height.
  private final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(1.0, 1.8);
  private final LoggedMechanismLigament2d carriageLine;

  /** Builds the elevator with the real hardware. */
  public ElevatorSubsystem() {
    this(new ElevatorIOTalonFX());
  }

  /** Builds the elevator with any hardware layer: real, simulated, or one that does nothing. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    LoggedMechanismRoot2d root =
        mechanism2d.getRoot("Elevator", 0.5, ElevatorConstants.BASE_BOTTOM_Z_METERS);
    root.append(
        new LoggedMechanismLigament2d(
            "Base", ElevatorConstants.BASE_HEIGHT_METERS, 90, 10, new Color8Bit(Color.kOrange)));
    carriageLine =
        root.append(
            new LoggedMechanismLigament2d(
                "Carriage",
                ElevatorConstants.CARRIAGE_HEIGHT_METERS,
                90,
                6,
                new Color8Bit(Color.kBlue)));
  }

  /** Runs every 20 ms: read the hardware, check for problems, decide, then log. */
  @Override
  public void periodic() {
    // Read the hardware first, so every decision below uses this loop's values.
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Elevator/IO", inputs);

    disconnectedAlert.set(!inputs.motorConnected);
    homeOnSwitchPress();
    sendGainsIfChanged();

    if (DriverStation.isDisabled()) {
      // Don't resume an old goal or voltage when re-enabled.
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

    updateNotArrivedAlert();
    logOutputs();
  }

  /** Drives down slowly until the bottom switch closes, or gives up after the timeout. */
  private void runHoming() {
    homingLoops++;
    if (inputs.bottomLimitPressed) {
      // homeOnSwitchPress() has already set the position to 0. Now rest on the bottom.
      setGoal(ElevatorConstants.STOW_HEIGHT_METERS);
      io.stop();
    } else if (homingLoops >= ElevatorConstants.HOMING_TIMEOUT_LOOPS) {
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
    if (mode == ControlMode.POSITION && !atGoal()) {
      notArrivedLoops++;
    } else {
      notArrivedLoops = 0;
    }
    notArrivedAlert.set(notArrivedLoops >= ElevatorConstants.NOT_ARRIVED_TIMEOUT_LOOPS);
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
    double[] gains = {kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get()};
    if (Arrays.equals(gains, sentGains)) {
      return;
    }
    System.arraycopy(gains, 0, sentGains, 0, gains.length);
    io.setGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
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
    carriageLine.setLength(ElevatorConstants.CARRIAGE_HEIGHT_METERS + heightMeters);
    Logger.recordOutput("Mech/Elevator/GoalMeters", goalMeters);
    Logger.recordOutput("Mech/Elevator/ControlMode", mode.name());
    Logger.recordOutput("Mech/Elevator/Homed", homed);
    Logger.recordOutput("Mech/Elevator/AtGoal", atGoal());
    Logger.recordOutput("Mech/Elevator/ComponentPoses", componentPoses(heightMeters));
    Logger.recordOutput("Mech/Elevator/ComponentPosesGoal", componentPoses(goalMeters));
    Logger.recordOutput("Mech/Elevator/Mechanism2d", mechanism2d);
  }

  /**
   * Where each moving part of the 3D model is, for a carriage height. The model's parts are
   * exported with the elevator fully down, so each pose is just how far that part has risen. In a
   * cascade, stage 1 rises half as far as the carriage (1 / RIGGING_RATIO).
   */
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
    homingLoops = 0;
    homingFailed = false;
    homingFailedAlert.set(false);
  }

  /** Moves to a height and holds it there. Needs homing first. */
  public void setGoal(double heightMeters) {
    mode = ControlMode.POSITION;
    goalMeters = clampHeight(heightMeters);
    notArrivedLoops = 0;
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
