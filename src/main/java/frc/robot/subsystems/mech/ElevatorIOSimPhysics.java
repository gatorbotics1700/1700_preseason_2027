package frc.robot.subsystems.mech;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The simulator's elevator: runs the real ElevatorIOTalonFX code against a simulated TalonFX and a
 * WPILib ElevatorSim physics model. The cascade is modeled as one mass on one drum.
 */
public class ElevatorIOSimPhysics extends ElevatorIOTalonFX {
  /** Carriage height that counts as resting on the bottom limit switch. */
  public static final double LIMIT_SWITCH_TRIGGER_METERS = 0.002;

  /** The real top hard stop sits a little above the soft limit. */
  public static final double TOP_HARD_STOP_METERS = ElevatorConstants.MAX_HEIGHT_METERS + 0.02;

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  // Cascade referred to the carriage: gravity sees m_carriage + m_stage1 / R, drum radius is R x.
  public static final double EFFECTIVE_MASS_KG =
      ElevatorConstants.CARRIAGE_MASS_KG
          + ElevatorConstants.STAGE_1_MASS_KG / ElevatorConstants.RIGGING_RATIO;
  public static final double EFFECTIVE_DRUM_RADIUS_METERS =
      ElevatorConstants.DRUM_PITCH_DIAMETER_METERS / 2.0 * ElevatorConstants.RIGGING_RATIO;

  // Faster than the 20 ms robot loop, so the TalonFX's 1 kHz loop doesn't see stale positions.
  private static final double SIM_PERIOD_SECONDS = 0.005;

  private final ElevatorSim physics;
  private final DIOSim limitSim;
  private final Notifier simThread;
  private double lastSimTime;

  /** The encoder reads 0 at boot wherever the carriage really is, like the real robot. */
  private final double bootHeightMeters;

  /** Builds the sim with the carriage at startHeightMeters; above the switch means un-homed. */
  public ElevatorIOSimPhysics(double startHeightMeters) {
    super();
    bootHeightMeters = startHeightMeters;
    physics = createPhysics(startHeightMeters);
    limitSim = new DIOSim(bottomLimit);
    motor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    lastSimTime = Utils.getCurrentTimeSeconds();
    simThread = new Notifier(this::stepPhysics);
    simThread.setName("ElevatorSim");
    simThread.startPeriodic(SIM_PERIOD_SECONDS);
  }

  /** The plant model, shared with ElevatorPhysicsTest. */
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

  /** Advances the plant by the real time since the last step, then feeds the sensors. */
  private synchronized void stepPhysics() {
    double now = Utils.getCurrentTimeSeconds();
    double dt = now - lastSimTime;
    lastSimTime = now;

    TalonFXSimState simState = motor.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    physics.setInputVoltage(simState.getMotorVoltage());
    physics.update(dt);

    double travelSinceBoot = physics.getPositionMeters() - bootHeightMeters;
    simState.setRawRotorPosition(travelSinceBoot * ElevatorConstants.ROTOR_ROTATIONS_PER_METER);
    simState.setRotorVelocity(
        physics.getVelocityMetersPerSecond() * ElevatorConstants.ROTOR_ROTATIONS_PER_METER);

    boolean pressed = physics.getPositionMeters() <= LIMIT_SWITCH_TRIGGER_METERS;
    limitSim.setValue(
        pressed
            ? ElevatorConstants.LIMIT_SWITCH_PRESSED_VALUE
            : !ElevatorConstants.LIMIT_SWITCH_PRESSED_VALUE);
  }

  /** Reads the simulated motor and also logs the true height. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    Logger.recordOutput("Mech/Elevator/Sim/TrueHeightMeters", getTrueHeightMeters());
  }

  /** Ground truth carriage height from the physics model (for logging and tests only). */
  public synchronized double getTrueHeightMeters() {
    return physics.getPositionMeters();
  }
}
