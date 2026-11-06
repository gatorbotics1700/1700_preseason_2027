package frc.robot.subsystems.template;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * TEMPLATE: example tests. Each test builds a fresh subsystem with TemplateIOSim, calls periodic()
 * directly, and checks the answer right away, with no waiting.
 *
 * <p>Expected numbers are typed here, not read from the code's constants, so an accidental change
 * makes a test fail. After writing a test, break the code on purpose once and check that it fails.
 */
public class TemplateSubsystemTest {

  private static final double TEST_SPEED = 0.5;

  private TemplateIOSim io;
  private TemplateSubsystem subsystem;

  /** Starts WPILib once, so alerts and logging work in tests. */
  @BeforeAll
  public static void startWpilib() {
    HAL.initialize(500, 0);
  }

  /** Builds a fresh subsystem with pretend hardware before each test. */
  @BeforeEach
  public void buildSubsystem() {
    io = new TemplateIOSim();
    subsystem = new TemplateSubsystem(io);
  }

  /** Forgets the subsystem after each test. */
  @AfterEach
  public void cleanUp() {
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  /** A speed sent to the subsystem reaches the motor. */
  @Test
  public void speedReachesMotor() {
    subsystem.setSpeed(TEST_SPEED);

    assertEquals(TEST_SPEED, io.getSpeed());
  }

  /** Stopping sends speed 0 to the motor. */
  @Test
  public void stopSendsZero() {
    subsystem.setSpeed(TEST_SPEED);
    subsystem.stop();

    assertEquals(0, io.getSpeed());
  }

  /** Unplugging the motor turns on the alert, and plugging it back turns it off. */
  @Test
  public void disconnectedMotorTurnsOnAlert() {
    subsystem.periodic();
    assertFalse(subsystem.motorDisconnectedAlert.get());

    io.setConnected(false);
    subsystem.periodic();
    assertTrue(subsystem.motorDisconnectedAlert.get());

    io.setConnected(true);
    subsystem.periodic();
    assertFalse(subsystem.motorDisconnectedAlert.get());
  }
}
