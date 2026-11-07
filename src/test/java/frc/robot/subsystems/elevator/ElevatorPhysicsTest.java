package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Feedforward constants must agree with the physics model the sim uses. */
class ElevatorPhysicsTest {
  private static final DCMotor M = ElevatorIOSim.GEARBOX;
  private static final double G = 9.81;
  private static final double R_EFF = ElevatorIOSim.EFFECTIVE_DRUM_RADIUS_METERS;

  /** ElevatorSim reads the simulated battery voltage, which needs WPILib's HAL started. */
  @BeforeAll
  static void startHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  private static void assertWithin(double expected, double actual, double frac, String what) {
    assertEquals(
        expected,
        actual,
        Math.abs(expected) * frac,
        what + ": expected ~" + expected + " but constant is " + actual);
  }

  @Test
  void kGMatchesHoldingVoltage() {
    double motorTorque = ElevatorIOSim.EFFECTIVE_MASS_KG * G * R_EFF / GEAR_RATIO;
    double volts = motorTorque / M.KtNMPerAmp * M.rOhms;
    assertWithin(volts, KG, 0.10, "kG");
  }

  @Test
  void kVMatchesBackEmf() {
    double rotorRadPerMeter = ROTOR_ROTATIONS_PER_METER * 2 * Math.PI;
    assertWithin(rotorRadPerMeter / M.KvRadPerSecPerVolt, KV, 0.10, "kV");
  }

  @Test
  void kAMatchesInertia() {
    double inertialMass = CARRIAGE_MASS_KG + STAGE_1_MASS_KG / (RIGGING_RATIO * RIGGING_RATIO);
    double motorTorquePerMps2 = inertialMass * R_EFF / GEAR_RATIO;
    assertWithin(motorTorquePerMps2 / M.KtNMPerAmp * M.rOhms, KA, 0.10, "kA");
  }

  @Test
  void kGHoldsTheCarriageInThePlant() {
    ElevatorSim plant = ElevatorIOSim.createPhysics(0.5);
    for (int i = 0; i < 50; i++) {
      plant.setInputVoltage(KG);
      plant.update(0.02);
    }
    // the plant's gravity uses the gravity-exact mass, so kG should hold to within a few mm
    assertEquals(0.5, plant.getPositionMeters(), 0.01, "carriage drifted under kG alone");
  }

  @Test
  void gearingCanReachTheTopQuickly() {
    ElevatorSim plant = ElevatorIOSim.createPhysics(0.0);
    double t = 0;
    while (plant.getPositionMeters() < MAX_HEIGHT_METERS && t < 2.0) {
      plant.setInputVoltage(12);
      plant.update(0.02);
      t += 0.02;
    }
    assertTrue(t < 1.0, "full travel at 12 V took " + t + " s; gearing too slow for the profile");
  }

  @Test
  void statorLimitLeavesHeadroomForWorstCaseDeceleration() {
    // Worst case is slowing down while moving down: the motor fights gravity plus deceleration.
    // Phoenix's motor model reports ~30% more stator current than WPILib's for the same torque,
    // so require 40% headroom or Motion Magic gets current limited and overshoots (seen in sim).
    double torque = ElevatorIOSim.EFFECTIVE_MASS_KG * (G + ACCELERATION_MPS2) * R_EFF / GEAR_RATIO;
    double amps = torque / M.KtNMPerAmp;
    assertTrue(
        amps <= 0.6 * STATOR_CURRENT_LIMIT_AMPS,
        "worst-case decel needs "
            + amps
            + " A against a "
            + STATOR_CURRENT_LIMIT_AMPS
            + " A limit");
  }

  @Test
  void profileIsAchievable() {
    // the cruise velocity must be under ~85% of free speed at 12 V so Motion Magic can track it
    double freeSpeedMps = 12.0 / KV;
    assertTrue(CRUISE_VELOCITY_MPS < 0.85 * freeSpeedMps, "cruise velocity above free speed");
  }
}
