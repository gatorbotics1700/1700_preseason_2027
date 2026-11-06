package frc.robot.subsystems.elevator;

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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Two-stage cascade elevator.
 *
 * <p>Positions are carriage height in meters above the bottom hard stop. The elevator refuses
 * position control until it has been homed (the bottom limit switch has been seen pressed), since
 * the encoder reads 0 at boot wherever the carriage actually is.
 *
 * <p>For AdvantageScope it logs:
 *
 * <ul>
 *   <li>Mech/Elevator/ComponentPoses: Pose3d[] for the 3D robot model's components (0 = stage 1, 1
 *       = carriage), matching ascope_assets/Robot_GatorboticsElevator
 *   <li>Mech/Elevator/ComponentPosesGoal: the same for the goal, for a ghost overlay
 *   <li>Mech/Elevator/Mechanism2d: a 2D side view
 * </ul>
 */
public class ElevatorSubsystem extends SubsystemBase {
  public enum ControlMode {
    IDLE,
    POSITION,
    VOLTAGE
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ControlMode mode = ControlMode.IDLE;
  private double goalMeters = ElevatorConstants.STOW_HEIGHT_METERS;
  private double voltageOutput = 0.0;
  private boolean homed = false;
  private boolean limitWasPressed = false;

  private final Alert disconnectedAlert =
      new Alert("Elevator motor disconnected", AlertType.kError);

  // Tunable gains, shown in AdvantageScope under /Tuning/Elevator
  private static final LoggedNetworkNumber kP =
      new LoggedNetworkNumber("/Tuning/Elevator/kP", ElevatorConstants.KP);
  private static final LoggedNetworkNumber kI =
      new LoggedNetworkNumber("/Tuning/Elevator/kI", ElevatorConstants.KI);
  private static final LoggedNetworkNumber kD =
      new LoggedNetworkNumber("/Tuning/Elevator/kD", ElevatorConstants.KD);
  private static final LoggedNetworkNumber kS =
      new LoggedNetworkNumber("/Tuning/Elevator/kS", ElevatorConstants.KS);
  private static final LoggedNetworkNumber kG =
      new LoggedNetworkNumber("/Tuning/Elevator/kG", ElevatorConstants.KG);
  private static final LoggedNetworkNumber kV =
      new LoggedNetworkNumber("/Tuning/Elevator/kV", ElevatorConstants.KV);
  private static final LoggedNetworkNumber kA =
      new LoggedNetworkNumber("/Tuning/Elevator/kA", ElevatorConstants.KA);
  private final double[] appliedGains = {
    ElevatorConstants.KP,
    ElevatorConstants.KI,
    ElevatorConstants.KD,
    ElevatorConstants.KS,
    ElevatorConstants.KG,
    ElevatorConstants.KV,
    ElevatorConstants.KA
  };

  // 2D side view: fixed stage, then the carriage ligament grows with height
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.8);
  private final LoggedMechanismLigament2d carriageLigament;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    LoggedMechanismRoot2d root =
        mechanism.getRoot("Elevator", 0.5, ElevatorConstants.BASE_BOTTOM_Z_METERS);
    root.append(
        new LoggedMechanismLigament2d(
            "Base", ElevatorConstants.BASE_HEIGHT_METERS, 90, 10, new Color8Bit(Color.kOrange)));
    carriageLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Carriage",
                ElevatorConstants.CARRIAGE_HEIGHT_METERS,
                90,
                6,
                new Color8Bit(Color.kBlue)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Elevator", inputs);
    disconnectedAlert.set(!inputs.motorConnected);

    // The bottom limit switch is the only absolute reference: re-zero every time it closes.
    if (inputs.bottomLimitPressed && !limitWasPressed) {
      io.resetPosition(ElevatorConstants.MIN_HEIGHT_METERS);
      homed = true;
    }
    limitWasPressed = inputs.bottomLimitPressed;

    applyTunableGains();

    if (DriverStation.isDisabled()) {
      // Don't resume an old goal or voltage when re-enabled.
      mode = ControlMode.IDLE;
    }

    switch (mode) {
      case POSITION -> {
        boolean goalIsBottom =
            goalMeters
                <= ElevatorConstants.MIN_HEIGHT_METERS
                    + ElevatorConstants.POSITION_TOLERANCE_METERS;
        if (!homed || (goalIsBottom && inputs.bottomLimitPressed)) {
          // Not homed: refuse to move. Resting on the stop: neutral, no holding current.
          io.stop();
        } else if (goalIsBottom
            && inputs.positionMeters <= ElevatorConstants.LANDING_HEIGHT_METERS + 0.005) {
          // Last few cm: set it down gently instead of driving the profile into the stop.
          io.setVoltage(ElevatorConstants.LANDING_VOLTS);
        } else if (goalIsBottom) {
          io.setPositionSetpoint(ElevatorConstants.LANDING_HEIGHT_METERS);
        } else {
          io.setPositionSetpoint(goalMeters);
        }
      }
      case VOLTAGE -> {
        boolean drivingIntoBottom = voltageOutput < 0 && inputs.bottomLimitPressed;
        if (drivingIntoBottom) {
          io.stop();
        } else {
          io.setVoltage(voltageOutput);
        }
      }
      case IDLE -> io.stop();
    }

    double height = getPositionMeters();
    carriageLigament.setLength(ElevatorConstants.CARRIAGE_HEIGHT_METERS + height);
    Logger.recordOutput("Mech/Elevator/GoalMeters", goalMeters);
    Logger.recordOutput("Mech/Elevator/ControlMode", mode.name());
    Logger.recordOutput("Mech/Elevator/Homed", homed);
    Logger.recordOutput("Mech/Elevator/AtGoal", atGoal());
    Logger.recordOutput("Mech/Elevator/ComponentPoses", componentPoses(height));
    Logger.recordOutput("Mech/Elevator/ComponentPosesGoal", componentPoses(goalMeters));
    Logger.recordOutput("Mech/Elevator/Mechanism2d", mechanism);
  }

  private void applyTunableGains() {
    double[] gains = {kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get()};
    if (!java.util.Arrays.equals(gains, appliedGains)) {
      System.arraycopy(gains, 0, appliedGains, 0, gains.length);
      io.setGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
    }
  }

  /**
   * Poses of the moving parts of the 3D model, in the robot frame, for a given carriage height. The
   * models are exported in place with the elevator fully down, so each pose is just how far that
   * part has risen. In a cascade the first stage rises 1/RIGGING_RATIO as far as the carriage.
   */
  public static Pose3d[] componentPoses(double carriageHeightMeters) {
    return new Pose3d[] {
      new Pose3d(0, 0, carriageHeightMeters / ElevatorConstants.RIGGING_RATIO, new Rotation3d()),
      new Pose3d(0, 0, carriageHeightMeters, new Rotation3d())
    };
  }

  /** Limits a requested height to the safe travel range. */
  public static double clampHeight(double heightMeters) {
    return MathUtil.clamp(
        heightMeters, ElevatorConstants.MIN_HEIGHT_METERS, ElevatorConstants.MAX_HEIGHT_METERS);
  }

  /** Moves to and holds a carriage height (clamped to the safe range). Needs homing first. */
  public void setGoal(double heightMeters) {
    mode = ControlMode.POSITION;
    goalMeters = clampHeight(heightMeters);
  }

  /** Drives open loop. Negative is down; it stops itself on the bottom limit switch. */
  public void setVoltage(double volts) {
    mode = ControlMode.VOLTAGE;
    voltageOutput = volts;
  }

  public void stop() {
    mode = ControlMode.IDLE;
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public double getGoalMeters() {
    return goalMeters;
  }

  public ControlMode getControlMode() {
    return mode;
  }

  public boolean isHomed() {
    return homed;
  }

  public boolean isBottomLimitPressed() {
    return inputs.bottomLimitPressed;
  }

  /** True when homed, holding a position, and within tolerance of the goal. */
  public boolean atGoal() {
    return homed
        && mode == ControlMode.POSITION
        && Math.abs(inputs.positionMeters - goalMeters)
            <= ElevatorConstants.POSITION_TOLERANCE_METERS;
  }
}
