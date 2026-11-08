package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Feedforward constants must agree with the physics model the sim uses. */
public class ElevatorPhysicsTest {
  private static final DCMotor MOTOR = ElevatorIOSimPhysics.GEARBOX;
  private static final double GRAVITY_MPS2 = 9.81;
  private static final double DRUM_RADIUS_METERS =
      ElevatorIOSimPhysics.EFFECTIVE_DRUM_RADIUS_METERS;

  /** ElevatorSim reads the simulated battery voltage, which needs WPILib's HAL started. */
  @BeforeAll
  public static void startHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  /** Fails unless actual is within a fraction (0.10 = 10%) of expected. */
  private static void assertWithin(double expected, double actual, double frac, String what) {
    assertEquals(
        expected,
        actual,
        Math.abs(expected) * frac,
        what + ": expected ~" + expected + " but constant is " + actual);
  }

  /** kG is the voltage physics says holds the carriage up, within 10%. */
  @Test
  public void kGMatchesHoldingVoltage() {
    double motorTorque =
        ElevatorIOSimPhysics.EFFECTIVE_MASS_KG
            * GRAVITY_MPS2
            * DRUM_RADIUS_METERS
            / ElevatorConstants.GEAR_RATIO;
    double volts = motorTorque / MOTOR.KtNMPerAmp * MOTOR.rOhms;
    assertWithin(volts, ElevatorConstants.KG, 0.10, "kG");
  }

  /** kV matches the motor's speed per volt and the gearing, within 10%. */
  @Test
  public void kVMatchesBackEmf() {
    double rotorRadPerMeter = ElevatorConstants.ROTOR_ROTATIONS_PER_METER * 2 * Math.PI;
    assertWithin(rotorRadPerMeter / MOTOR.KvRadPerSecPerVolt, ElevatorConstants.KV, 0.10, "kV");
  }

  /** kA matches the moving mass and the gearing, within 10%. */
  @Test
  public void kAMatchesInertia() {
    double inertialMass =
        ElevatorConstants.CARRIAGE_MASS_KG
            + ElevatorConstants.STAGE_1_MASS_KG
                / (ElevatorConstants.RIGGING_RATIO * ElevatorConstants.RIGGING_RATIO);
    double motorTorquePerMps2 = inertialMass * DRUM_RADIUS_METERS / ElevatorConstants.GEAR_RATIO;
    assertWithin(
        motorTorquePerMps2 / MOTOR.KtNMPerAmp * MOTOR.rOhms, ElevatorConstants.KA, 0.10, "kA");
  }

  /** In the physics model, kG alone holds the carriage still. */
  @Test
  public void kGHoldsTheCarriageInThePlant() {
    ElevatorSim plant = ElevatorIOSimPhysics.createPhysics(0.5);
    for (int i = 0; i < 50; i++) {
      plant.setInputVoltage(ElevatorConstants.KG);
      plant.update(0.02);
    }
    assertEquals(0.5, plant.getPositionMeters(), 0.01, "carriage drifted under kG alone");
  }

  /** At full voltage the carriage reaches the top in under 1 s. */
  @Test
  public void gearingCanReachTheTopQuickly() {
    ElevatorSim plant = ElevatorIOSimPhysics.createPhysics(0.0);
    double t = 0;
    while (plant.getPositionMeters() < ElevatorConstants.MAX_HEIGHT_METERS && t < 2.0) {
      plant.setInputVoltage(12);
      plant.update(0.02);
      t += 0.02;
    }
    assertTrue(t < 1.0, "full travel at 12 V took " + t + " s; gearing too slow for the profile");
  }

  /** The current limit leaves room for the hardest stop: slowing down while moving down. */
  @Test
  public void statorLimitLeavesHeadroomForWorstCaseDeceleration() {
    // Phoenix reports about 30% more stator current than WPILib's model, so keep 40% headroom.
    double torque =
        ElevatorIOSimPhysics.EFFECTIVE_MASS_KG
            * (GRAVITY_MPS2 + ElevatorConstants.ACCELERATION_MPS2)
            * DRUM_RADIUS_METERS
            / ElevatorConstants.GEAR_RATIO;
    double amps = torque / MOTOR.KtNMPerAmp;
    assertTrue(
        amps <= 0.6 * ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS,
        "worst-case decel needs "
            + amps
            + " A against a "
            + ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS
            + " A limit");
  }

  /** The Motion Magic cruise speed is below what the motor can reach. */
  @Test
  public void profileIsAchievable() {
    double freeSpeedMps = 12.0 / ElevatorConstants.KV;
    assertTrue(
        ElevatorConstants.CRUISE_VELOCITY_MPS < 0.85 * freeSpeedMps,
        "cruise velocity above free speed");
  }
}
