package frc.robot.subsystems.mech;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

// TODO - TEMP CODE COPIED FROM SIMON
// Follows ElevatorIOSimPhysics from the elevator prototype (PR #8), changed for a swinging arm.
// When that PR lands, keep this class, but compare it with his and match any fixes.

/**
 * The simulator's intake: runs the real IntakeIOTalonFX code against CTRE's simulated motors and a
 * WPILib arm physics model, so the deploy arm takes time to move and the limit switches click on
 * their own. The rollers are not simulated: they spin "in the air", with no fuel.
 *
 * <p>Two ways of measuring the arm's angle meet here:
 *
 * <ul>
 *   <li>Intake angle: what the robot code uses. 0 = retracted, 95 = deployed.
 *   <li>Physics angle: what SingleJointedArmSim uses. 0 = level with the ground, 90 = straight up.
 * </ul>
 */
public class IntakeIOSimPhysics extends IntakeIOTalonFX {

  // TODO - PLACEHOLDER. Replace with actual values when we have them from CAD
  // Assumes Kraken X60 motors. Check with the build team which motors the arm uses.
  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(IntakeConstants.DEPLOY_MOTOR_COUNT);

  // The real arm can go a little past each limit switch before hitting a hard stop.
  private static final double HARD_STOP_PAST_SWITCH_DEGREES = 2;
  // A switch counts as pressed when the arm is within this many degrees of its end.
  private static final double SWITCH_PRESSED_WITHIN_DEGREES = 1;

  // Faster than the 20 ms robot loop, so the TalonFX's own fast loop sees fresh positions.
  private static final double SIM_PERIOD_SECONDS = 0.005;

  private final SingleJointedArmSim physics;
  private final DIOSim retractedSwitchSim;
  private final DIOSim deployedSwitchSim;
  private final Notifier simThread;
  private double lastSimTimeSeconds;

  /** Builds the sim with the arm retracted, which is how the robot code assumes it starts. */
  public IntakeIOSimPhysics() {
    super();
    physics = createPhysics(IntakeConstants.RETRACTED_ANGLE_DEGREES);
    retractedSwitchSim = new DIOSim(retractedLimitSwitch);
    deployedSwitchSim = new DIOSim(deployedLimitSwitch);

    // The deploy motors are set to "clockwise is positive", so the simulated motors match that.
    leftDeployMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    rightDeployMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    lastSimTimeSeconds = Utils.getCurrentTimeSeconds();
    simThread = new Notifier(this::stepPhysics);
    simThread.setName("IntakeSim");
    simThread.startPeriodic(SIM_PERIOD_SECONDS);
  }

  /** The arm's physics model, starting at this intake angle. Also used by IntakePhysicsTest. */
  public static SingleJointedArmSim createPhysics(double startIntakeDegrees) {
    double minIntakeDegrees =
        IntakeConstants.RETRACTED_ANGLE_DEGREES - HARD_STOP_PAST_SWITCH_DEGREES;
    double maxIntakeDegrees =
        IntakeConstants.EXTENDED_ANGLE_DEGREES + HARD_STOP_PAST_SWITCH_DEGREES;
    double momentOfInertia =
        SingleJointedArmSim.estimateMOI(
            IntakeConstants.ARM_PIVOT_TO_CENTER_OF_MASS_METERS, IntakeConstants.ARM_MASS_KG);
    // Deploying makes the physics angle smaller, so the deployed end is the physics minimum.
    return new SingleJointedArmSim(
        GEARBOX,
        IntakeConstants.DEPLOY_TOTAL_GEAR_RATIO,
        momentOfInertia,
        IntakeConstants.ARM_PIVOT_TO_CENTER_OF_MASS_METERS,
        intakeDegreesToPhysicsRadians(maxIntakeDegrees),
        intakeDegreesToPhysicsRadians(minIntakeDegrees),
        true,
        intakeDegreesToPhysicsRadians(startIntakeDegrees));
  }

  /** Turns an intake angle (0 = retracted) into a physics angle (0 = level), in radians. */
  public static double intakeDegreesToPhysicsRadians(double intakeDegrees) {
    return Math.toRadians(IntakeConstants.RETRACTED_ANGLE_ABOVE_LEVEL_DEGREES - intakeDegrees);
  }

  /** Turns a physics angle (0 = level), in radians, back into an intake angle (0 = retracted). */
  public static double physicsRadiansToIntakeDegrees(double physicsRadians) {
    return IntakeConstants.RETRACTED_ANGLE_ABOVE_LEVEL_DEGREES - Math.toDegrees(physicsRadians);
  }

  /** True when an arm at this intake angle presses the retracted switch. */
  public static boolean isRetractedSwitchPressed(double intakeDegrees) {
    return intakeDegrees <= IntakeConstants.RETRACTED_ANGLE_DEGREES + SWITCH_PRESSED_WITHIN_DEGREES;
  }

  /** True when an arm at this intake angle presses the deployed switch. */
  public static boolean isDeployedSwitchPressed(double intakeDegrees) {
    return intakeDegrees >= IntakeConstants.EXTENDED_ANGLE_DEGREES - SWITCH_PRESSED_WITHIN_DEGREES;
  }

  /** Moves the physics forward by the time since the last step, then updates the sensors. */
  private synchronized void stepPhysics() {
    double nowSeconds = Utils.getCurrentTimeSeconds();
    double stepSeconds = nowSeconds - lastSimTimeSeconds;
    lastSimTimeSeconds = nowSeconds;

    TalonFXSimState leftSim = leftDeployMotor.getSimState();
    TalonFXSimState rightSim = rightDeployMotor.getSimState();
    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Both motors get the same command, so the left one's voltage stands for both.
    // Positive motor voltage deploys the arm, which makes the physics angle smaller, hence the
    // minus.
    physics.setInputVoltage(-leftSim.getMotorVoltage());
    physics.update(stepSeconds);

    updateSimulatedMotors(leftSim, rightSim);
    updateSimulatedSwitches();
  }

  /** Tells both simulated motors where the arm is now, in motor turns. */
  private void updateSimulatedMotors(TalonFXSimState leftSim, TalonFXSimState rightSim) {
    double armTurns = getTrueIntakeDegrees() / 360.0;
    double armTurnsPerSecond = -Math.toDegrees(physics.getVelocityRadPerSec()) / 360.0;
    double motorTurns = armTurns * IntakeConstants.DEPLOY_TOTAL_GEAR_RATIO;
    double motorTurnsPerSecond = armTurnsPerSecond * IntakeConstants.DEPLOY_TOTAL_GEAR_RATIO;
    leftSim.setRawRotorPosition(motorTurns);
    rightSim.setRawRotorPosition(motorTurns);
    leftSim.setRotorVelocity(motorTurnsPerSecond);
    rightSim.setRotorVelocity(motorTurnsPerSecond);
  }

  /** Presses each limit switch when the arm reaches that end. get() == true means pressed. */
  private void updateSimulatedSwitches() {
    double intakeDegrees = getTrueIntakeDegrees();
    retractedSwitchSim.setValue(isRetractedSwitchPressed(intakeDegrees));
    deployedSwitchSim.setValue(isDeployedSwitchPressed(intakeDegrees));
  }

  /** Reads the simulated hardware, and also logs where the physics says the arm really is. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);
    Logger.recordOutput("Mech/Intake/Sim/TrueAngleDegrees", getTrueIntakeDegrees());
  }

  /** The arm's true intake angle from the physics model (for logging and tests only). */
  public synchronized double getTrueIntakeDegrees() {
    return physicsRadiansToIntakeDegrees(physics.getAngleRads());
  }
}
