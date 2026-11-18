package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

// TODO - TEMP CODE COPIED FROM SIMON
// Follows ElevatorPhysicsTest from the elevator prototype (PR #8), changed for a swinging arm.

/**
 * Checks the intake's physics model (IntakeIOSimPhysics) without running the robot: the angle
 * conversions, the limit switch rules, and that gravity pulls the arm the right way.
 */
public class IntakePhysicsTest {

  private static final double DELTA = 1e-9;
  private static final double STEP_SECONDS = 0.005;
  private static final int STEPS_IN_ONE_SECOND = 200;

  /**
   * Starts WPILib's HAL (its link to the hardware, simulated here). The physics model reads the
   * simulated battery voltage, which needs the HAL.
   */
  @BeforeAll
  public static void startHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  /**
   * Runs the physics for one simulated second with this motor voltage. Positive deploys, like on
   * the robot; the physics angle runs the other way, hence the minus.
   */
  private static double runForOneSecond(SingleJointedArmSim physics, double motorVolts) {
    physics.setInputVoltage(-motorVolts);
    for (int i = 0; i < STEPS_IN_ONE_SECOND; i++) {
      physics.update(STEP_SECONDS);
    }
    return IntakeIOSimPhysics.physicsRadiansToIntakeDegrees(physics.getAngleRads());
  }

  /** Retracted (0) is straight up (90 above level); deployed (95) is 5 below level. */
  @Test
  public void anglesConvertBetweenIntakeAndPhysics() {
    assertEquals(Math.toRadians(90), IntakeIOSimPhysics.intakeDegreesToPhysicsRadians(0), DELTA);
    assertEquals(Math.toRadians(-5), IntakeIOSimPhysics.intakeDegreesToPhysicsRadians(95), DELTA);
    assertEquals(45, IntakeIOSimPhysics.physicsRadiansToIntakeDegrees(Math.toRadians(45)), DELTA);
  }

  /** Each switch is pressed only within 1 degree of its end of travel. */
  @Test
  public void switchesPressOnlyNearTheirEnd() {
    assertTrue(IntakeIOSimPhysics.isRetractedSwitchPressed(0));
    assertTrue(IntakeIOSimPhysics.isRetractedSwitchPressed(0.5));
    assertFalse(IntakeIOSimPhysics.isRetractedSwitchPressed(5));
    assertFalse(IntakeIOSimPhysics.isRetractedSwitchPressed(95));

    assertTrue(IntakeIOSimPhysics.isDeployedSwitchPressed(95));
    assertTrue(IntakeIOSimPhysics.isDeployedSwitchPressed(94.5));
    assertFalse(IntakeIOSimPhysics.isDeployedSwitchPressed(90));
    assertFalse(IntakeIOSimPhysics.isDeployedSwitchPressed(0));
  }

  /** Gravity pulls an unpowered arm, tipped a little past straight up, toward deployed. */
  @Test
  public void gravityPullsTippedArmTowardDeployed() {
    SingleJointedArmSim physics = IntakeIOSimPhysics.createPhysics(10);
    double intakeDegrees = runForOneSecond(physics, 0);
    // Slowly: with 0 volts the simulated motors act as a brake.
    assertTrue(intakeDegrees > 10, "arm should fall outward, but is at " + intakeDegrees);
  }

  /** An unpowered arm standing straight up (retracted) is balanced, so it stays put. */
  @Test
  public void unpoweredArmStraightUpStaysRetracted() {
    SingleJointedArmSim physics = IntakeIOSimPhysics.createPhysics(0);
    assertEquals(0, runForOneSecond(physics, 0), 1);
  }

  /** Positive volts deploy the arm to the hard stop, 2 degrees past the deployed switch. */
  @Test
  public void positiveVoltsDeployToHardStop() {
    SingleJointedArmSim physics = IntakeIOSimPhysics.createPhysics(0);
    double intakeDegrees = runForOneSecond(physics, 6);
    assertEquals(97, intakeDegrees, 0.5);
    assertTrue(IntakeIOSimPhysics.isDeployedSwitchPressed(intakeDegrees));
  }

  /** Negative volts retract the arm to the hard stop, 2 degrees past the retracted switch. */
  @Test
  public void negativeVoltsRetractToHardStop() {
    SingleJointedArmSim physics = IntakeIOSimPhysics.createPhysics(95);
    double intakeDegrees = runForOneSecond(physics, -6);
    assertEquals(-2, intakeDegrees, 0.5);
    assertTrue(IntakeIOSimPhysics.isRetractedSwitchPressed(intakeDegrees));
  }
}
