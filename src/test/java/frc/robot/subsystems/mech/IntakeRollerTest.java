package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Roller tests that use IntakeIOSim instead of simulated motors. IntakeIOSim just remembers what
 * the intake told it, so each test checks the answer right away, with no waiting. The deploy arm
 * tests are in IntakeSubsystemTest.
 */
public class IntakeRollerTest {

  private static final double REVERSE_SPEED = -IntakeConstants.INTAKING_SPEED;
  private static final double DASHBOARD_CURRENT_LIMIT_AMPS = 90; // intakeCurrentLimit's default

  // A brand-new intake for every test. This works because IntakeIOSim owns no real ports or
  // motors, so nothing from one test can leak into the next.
  private IntakeIOSim io;
  private IntakeSubsystem intake;

  /** Starts WPILib once, so the dashboard numbers can be created. */
  @BeforeAll
  public static void startWpilib() {
    HAL.initialize(500, 0);
  }

  /** Builds a fresh intake with pretend hardware before each test. */
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

  /** Intaking sends the intaking speed to the rollers. */
  @Test
  public void intakingSpeedReachesRollers() {
    intake.setIntakeSpeed(IntakeConstants.INTAKING_SPEED);

    assertEquals(IntakeConstants.INTAKING_SPEED, io.getRollerSpeed());
  }

  /** Reversing sends the opposite speed to the rollers. */
  @Test
  public void reverseSpeedReachesRollers() {
    intake.setIntakeSpeed(REVERSE_SPEED);

    assertEquals(REVERSE_SPEED, io.getRollerSpeed());
  }

  /** Speed 0 stops the rollers. */
  @Test
  public void zeroSpeedStopsRollers() {
    intake.setIntakeSpeed(IntakeConstants.INTAKING_SPEED);
    intake.setIntakeSpeed(0);

    assertEquals(0, io.getRollerSpeed());
  }

  /** After one loop, the dashboard's roller current limit has been sent to the rollers. */
  @Test
  public void dashboardCurrentLimitReachesRollers() {
    intake.periodic();

    assertEquals(DASHBOARD_CURRENT_LIMIT_AMPS, io.getRollerCurrentLimitAmps());
  }
}
