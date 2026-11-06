package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Physics sim of the elevator. It runs the real {@link ElevatorIOTalonFX} code against Phoenix's
 * simulated TalonFX, so the same onboard Motion Magic controller and config run in sim and on the
 * robot. WPILib's {@link ElevatorSim} turns the simulated motor voltage into carriage motion.
 *
 * <p>Cascade model: ElevatorSim has one mass on one drum. Referred to the carriage, gravity on a
 * cascade is (m_carriage + m_stage1 / R) g, and the drum moves the carriage R times its surface
 * travel, so we use that mass and drum radius * R. (Inertia is really m_carriage + m_stage1 / R^2,
 * so this slightly overestimates it, which is the conservative direction for tuning.)
 */
public class ElevatorIOSim extends ElevatorIOTalonFX {
  /** Carriage height that counts as resting on the bottom limit switch. */
  public static final double LIMIT_SWITCH_TRIGGER_METERS = 0.002;

  /** The real top hard stop sits a little above the soft limit. */
  public static final double TOP_HARD_STOP_METERS = ElevatorConstants.MAX_HEIGHT_METERS + 0.02;

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  public static final double EFFECTIVE_MASS_KG =
      ElevatorConstants.CARRIAGE_MASS_KG
          + ElevatorConstants.STAGE_1_MASS_KG / ElevatorConstants.RIGGING_RATIO;
  public static final double EFFECTIVE_DRUM_RADIUS_METERS =
      ElevatorConstants.DRUM_PITCH_DIAMETER_METERS / 2.0 * ElevatorConstants.RIGGING_RATIO;

  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final ElevatorSim physics;
  private final DIOSim limitSim;

  /** The encoder reads 0 at boot wherever the carriage really is, like the real robot. */
  private final double bootHeightMeters;

  /**
   * @param startHeightMeters where the carriage really is at boot. Anything above the limit switch
   *     means the robot boots un-homed and has to run the homing routine, like the real robot.
   */
  public ElevatorIOSim(double startHeightMeters) {
    super();
    bootHeightMeters = startHeightMeters;
    physics = createPhysics(startHeightMeters);
    limitSim = new DIOSim(bottomLimit);
    motor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
  }

  /** The plant model, shared with ElevatorPhysicsTest so the test exercises the same physics. */
  public static ElevatorSim createPhysics(double startHeightMeters) {
    return new ElevatorSim(
        GEARBOX,
        ElevatorConstants.GEAR_RATIO,
        EFFECTIVE_MASS_KG,
        EFFECTIVE_DRUM_RADIUS_METERS,
        ElevatorConstants.MIN_HEIGHT_METERS,
        TOP_HARD_STOP_METERS,
        true,
        startHeightMeters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    TalonFXSimState simState = motor.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    physics.setInputVoltage(simState.getMotorVoltage());
    physics.update(LOOP_PERIOD_SECONDS);

    double travelSinceBoot = physics.getPositionMeters() - bootHeightMeters;
    simState.setRawRotorPosition(travelSinceBoot * ElevatorConstants.ROTOR_ROTATIONS_PER_METER);
    simState.setRotorVelocity(
        physics.getVelocityMetersPerSecond() * ElevatorConstants.ROTOR_ROTATIONS_PER_METER);

    boolean pressed = physics.getPositionMeters() <= LIMIT_SWITCH_TRIGGER_METERS;
    limitSim.setValue(
        pressed
            ? ElevatorConstants.LIMIT_SWITCH_PRESSED_VALUE
            : !ElevatorConstants.LIMIT_SWITCH_PRESSED_VALUE);

    super.updateInputs(inputs);
    Logger.recordOutput("Mech/Elevator/Sim/TrueHeightMeters", physics.getPositionMeters());
  }

  /** Ground truth carriage height from the physics model (for logging and tests only). */
  public double getTrueHeightMeters() {
    return physics.getPositionMeters();
  }
}
