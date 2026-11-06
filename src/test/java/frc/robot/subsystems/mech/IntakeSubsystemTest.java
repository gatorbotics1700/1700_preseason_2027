package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Checks the deploy arm's decisions: when it moves, where it aims, when it stops, and how it homes.
 * Each test builds a fresh intake with IntakeIOSim, so the answers are known right away, with no
 * waiting.
 *
 * <p>The roller tests are in IntakeRollerTest.
 */
public class IntakeSubsystemTest {

  private static final double ANGLE_TOLERANCE_DEGREES = 0.01;
  private static final double TEST_ARM_SPEED = 0.3;
  private static final double ARM_MID_TRAVEL_DEGREES = 50;
  private static final double JAM_CURRENT_AMPS = 40; // IntakeSubsystem's stop rule: 40 A or more
  private static final double JUST_UNDER_JAM_CURRENT_AMPS = 39.9;
  private static final int A_FEW_LOOPS = 3;
  // Typed here on purpose (2 s / 20 ms), so a change to the intake's time limit makes a test fail.
  private static final int LOOPS_IN_TWO_SECONDS = 100;

  private IntakeIOSim io;
  private IntakeSubsystem intake;

  /** Starts WPILib once, so the dashboard numbers can be created. */
  @BeforeAll
  public static void startWpilib() {
    HAL.initialize(500, 0);
  }

  /** Builds a fresh intake with pretend hardware: arm at 0 degrees, no switch pressed. */
  @BeforeEach
  public void buildIntake() {
    io = new IntakeIOSim();
    intake = new IntakeSubsystem(io);
  }

  /** Forgets the intake after each test. */
  @AfterEach
  public void cleanUp() {
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  // ---------------- Helpers ----------------

  /** Runs the intake's 20 ms loop the given number of times. No real waiting. */
  private void runLoops(int loops) {
    for (int i = 0; i < loops; i++) {
      intake.periodic();
    }
  }

  /** Moves the pretend arm to an angle, as if it had swung there. */
  private void moveArmToDegrees(double angleDegrees) {
    io.moveArmToRotations(intake.degreesToRevs(angleDegrees));
  }

  /** True if the arm was last told to stop: not driving to a target, and speed 0. */
  private boolean isArmStopped() {
    return !io.isDeployDrivenToTarget() && io.getDeploySpeed() == 0;
  }

  // ---------------- Startup ----------------

  /** On startup the arm is retracted, aims for 0 degrees, and is told to stay still. */
  @Test
  public void startsRetractedAndStopped() {
    runLoops(A_FEW_LOOPS);

    assertFalse(intake.isDeployed());
    assertEquals(
        IntakeConstants.RETRACTED_ANGLE_DEGREES,
        intake.getDesiredAngle().getDegrees(),
        ANGLE_TOLERANCE_DEGREES);
    assertTrue(isArmStopped());
    assertEquals(0, io.getRollerSpeed());
  }

  // ---------------- Deploy arm: moving out ----------------

  /** Asking to deploy aims the arm at 95 degrees and drives it there with position control. */
  @Test
  public void deployAimsAtExtendedAngle() {
    intake.setDesiredDeployPosition(false);
    runLoops(1);

    assertTrue(intake.isDeployed());
    assertEquals(
        IntakeConstants.EXTENDED_ANGLE_DEGREES,
        intake.getDesiredAngle().getDegrees(),
        ANGLE_TOLERANCE_DEGREES);
    assertTrue(io.isDeployDrivenToTarget());
    assertEquals(
        intake.degreesToRevs(IntakeConstants.EXTENDED_ANGLE_DEGREES),
        io.getDeployTargetRotations());
  }

  /**
   * While deploying, pressing the deployed switch stops the arm in the same loop.
   *
   * <p>We check after ONE loop on purpose. Pressing the switch also resets the angle to 95 degrees,
   * so on the next loop the "within 3 degrees" rule would stop the arm anyway and hide a broken
   * switch rule.
   */
  @Test
  public void deployedSwitchStopsDeploying() {
    moveArmToDegrees(ARM_MID_TRAVEL_DEGREES);
    intake.setDesiredDeployPosition(false);
    runLoops(1);

    io.setDeployedSwitchPressed(true);
    runLoops(1);

    assertTrue(isArmStopped());
    assertTrue(intake.isDeployed());
  }

  /**
   * While deploying, getting within 3 degrees of the target counts as arrived, so the arm stops.
   */
  @Test
  public void deployStopsWithinDeadband() {
    intake.setDesiredDeployPosition(false);
    moveArmToDegrees(IntakeConstants.EXTENDED_ANGLE_DEGREES - 2);
    runLoops(1);

    assertTrue(isArmStopped());
  }

  /** While deploying, 4 degrees short of the target is not close enough, so the arm keeps going. */
  @Test
  public void deployKeepsGoingOutsideDeadband() {
    intake.setDesiredDeployPosition(false);
    moveArmToDegrees(IntakeConstants.EXTENDED_ANGLE_DEGREES - 4);
    runLoops(1);

    assertTrue(io.isDeployDrivenToTarget());
  }

  // ---------------- Deploy arm: moving in ----------------

  /** Asking to retract (from out) aims the arm at 0 degrees and drives it there. */
  @Test
  public void retractAimsAtRetractedAngle() {
    moveArmToDegrees(IntakeConstants.EXTENDED_ANGLE_DEGREES);
    intake.setDesiredDeployPosition(true);
    runLoops(1);

    assertFalse(intake.isDeployed());
    assertEquals(
        IntakeConstants.RETRACTED_ANGLE_DEGREES,
        intake.getDesiredAngle().getDegrees(),
        ANGLE_TOLERANCE_DEGREES);
    assertTrue(io.isDeployDrivenToTarget());
  }

  /**
   * While retracting, pressing the retracted switch stops the arm in the same loop. Checked after
   * ONE loop, for the same reason as deployedSwitchStopsDeploying.
   */
  @Test
  public void retractedSwitchStopsRetracting() {
    moveArmToDegrees(ARM_MID_TRAVEL_DEGREES);
    intake.setDesiredDeployPosition(true);
    runLoops(1);

    io.setRetractedSwitchPressed(true);
    runLoops(1);

    assertTrue(isArmStopped());
    assertFalse(intake.isDeployed());
  }

  // ---------------- Homing ----------------

  /**
   * Pressing the deployed switch resets the arm's angle reading to 95 degrees. Two loops: the first
   * resets the motors, the second reads the new angle back.
   */
  @Test
  public void deployedSwitchResetsAngleTo95() {
    moveArmToDegrees(40);
    io.setDeployedSwitchPressed(true);
    runLoops(2);

    assertEquals(
        IntakeConstants.EXTENDED_ANGLE_DEGREES,
        intake.getCurrentAngle().getDegrees(),
        ANGLE_TOLERANCE_DEGREES);
  }

  /** Pressing the retracted switch resets the arm's angle reading to 0 degrees. */
  @Test
  public void retractedSwitchResetsAngleTo0() {
    moveArmToDegrees(40);
    io.setRetractedSwitchPressed(true);
    runLoops(2);

    assertEquals(
        IntakeConstants.RETRACTED_ANGLE_DEGREES,
        intake.getCurrentAngle().getDegrees(),
        ANGLE_TOLERANCE_DEGREES);
  }

  // ---------------- Deploy arm: running at a speed ----------------

  /** A positive arm speed means "deploying", and the arm motors run forward at that speed. */
  @Test
  public void positiveArmSpeedMeansDeploying() {
    intake.setDeploySpeed(TEST_ARM_SPEED);
    runLoops(A_FEW_LOOPS);

    assertTrue(intake.isDeployed());
    assertEquals(TEST_ARM_SPEED, io.getDeploySpeed());
  }

  /** A negative arm speed means "retracting", and the arm motors run backward at that speed. */
  @Test
  public void negativeArmSpeedMeansRetracting() {
    moveArmToDegrees(ARM_MID_TRAVEL_DEGREES);
    intake.setDeploySpeed(-TEST_ARM_SPEED);
    runLoops(A_FEW_LOOPS);

    assertFalse(intake.isDeployed());
    assertEquals(-TEST_ARM_SPEED, io.getDeploySpeed());
  }

  /** An arm speed of 0 does not change whether the intake counts as deployed. */
  @Test
  public void zeroArmSpeedKeepsDeployedState() {
    intake.setDeploySpeed(TEST_ARM_SPEED);
    intake.setDeploySpeed(0);
    runLoops(A_FEW_LOOPS);

    assertTrue(intake.isDeployed());
  }

  // ---------------- Jam detection ----------------

  /**
   * At 40 A the arm counts as jammed and stops. This checks the RULE only. KNOWN ISSUE
   * (2026-11-01): the deploy motors' own current limit is also 40 A, so a real jammed motor hovers
   * around 40 A and may never reach it. The old simulator test read 40.2 A once and 38.96 A another
   * time.
   */
  @Test
  public void currentAtJamThresholdStopsArm() {
    intake.setDeploySpeed(TEST_ARM_SPEED);
    io.setDeployCurrentAmps(JAM_CURRENT_AMPS);
    runLoops(1);

    assertTrue(isArmStopped());
    assertTrue(intake.isDeployed()); // it counts as "deployed and stopped"
  }

  /** Just under 40 A is not a jam, so the arm keeps going. */
  @Test
  public void currentJustUnderJamThresholdKeepsGoing() {
    intake.setDeploySpeed(TEST_ARM_SPEED);
    io.setDeployCurrentAmps(JUST_UNDER_JAM_CURRENT_AMPS);
    runLoops(1);

    assertEquals(TEST_ARM_SPEED, io.getDeploySpeed());
  }

  // ---------------- Alerts ----------------

  /** With every motor connected, no "disconnected" alert is on. */
  @Test
  public void allMotorsConnectedMeansNoAlerts() {
    runLoops(1);

    assertFalse(intake.leftRollerDisconnectedAlert.get());
    assertFalse(intake.rightRollerDisconnectedAlert.get());
    assertFalse(intake.leftDeployDisconnectedAlert.get());
    assertFalse(intake.rightDeployDisconnectedAlert.get());
  }

  /** Unplugging one motor turns on that motor's alert only, and plugging it back turns it off. */
  @Test
  public void disconnectedMotorTurnsOnItsAlertOnly() {
    io.setLeftDeployConnected(false);
    runLoops(1);

    assertTrue(intake.leftDeployDisconnectedAlert.get());
    assertFalse(intake.leftRollerDisconnectedAlert.get());
    assertFalse(intake.rightRollerDisconnectedAlert.get());
    assertFalse(intake.rightDeployDisconnectedAlert.get());

    io.setLeftDeployConnected(true);
    runLoops(1);

    assertFalse(intake.leftDeployDisconnectedAlert.get());
  }

  /** An arm that never arrives is fine for 2 seconds (100 loops), then the alert turns on. */
  @Test
  public void armThatNeverArrivesTurnsOnAlert() {
    intake.setDesiredDeployPosition(false); // the pretend arm stays at 0 degrees
    runLoops(LOOPS_IN_TWO_SECONDS);
    assertFalse(intake.armNotArrivingAlert.get());

    runLoops(1);
    assertTrue(intake.armNotArrivingAlert.get());
  }

  /** An arm that arrives within 2 seconds never turns on the alert. */
  @Test
  public void armThatArrivesInTimeKeepsAlertOff() {
    intake.setDesiredDeployPosition(false);
    runLoops(LOOPS_IN_TWO_SECONDS / 2);
    moveArmToDegrees(IntakeConstants.EXTENDED_ANGLE_DEGREES);
    runLoops(LOOPS_IN_TWO_SECONDS);

    assertFalse(intake.armNotArrivingAlert.get());
  }

  /** Once the arm stops (here, at the deployed switch), the alert turns off. */
  @Test
  public void alertClearsWhenArmStops() {
    intake.setDesiredDeployPosition(false);
    runLoops(LOOPS_IN_TWO_SECONDS + 1);
    assertTrue(intake.armNotArrivingAlert.get());

    io.setDeployedSwitchPressed(true);
    runLoops(1);
    assertFalse(intake.armNotArrivingAlert.get());
  }
}
