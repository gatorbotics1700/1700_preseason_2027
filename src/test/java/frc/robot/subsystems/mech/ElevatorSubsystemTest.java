package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.mech.ElevatorIOSim.Request;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for the elevator's decisions, using ElevatorIOSim and calling periodic() directly. Expected
 * numbers are typed here, not read from ElevatorConstants, so an accidental change fails a test.
 */
public class ElevatorSubsystemTest {

  private static final double DELTA = 1e-9;

  private ElevatorIOSim io;
  private ElevatorSubsystem elevator;

  /** Starts WPILib once, so alerts, logging and the pretend Driver Station work in tests. */
  @BeforeAll
  public static void startWpilib() {
    HAL.initialize(500, 0);
  }

  /** Builds a fresh, enabled elevator with pretend hardware before each test. */
  @BeforeEach
  public void buildElevator() {
    setEnabled(true);
    io = new ElevatorIOSim();
    elevator = new ElevatorSubsystem(io);
  }

  /** Forgets the subsystem after each test. */
  @AfterEach
  public void cleanUp() {
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  /** Turns the pretend robot on or off. */
  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** Presses the bottom switch for one loop and lets go, so the elevator is homed at 0. */
  private void homeAtBottom() {
    io.setBottomLimitPressed(true);
    elevator.periodic();
    io.setBottomLimitPressed(false);
  }

  /** Before homing, a goal never moves the motor. */
  @Test
  public void refusesToMoveBeforeHoming() {
    elevator.setGoal(0.5);
    elevator.periodic();

    assertFalse(elevator.isHomed());
    assertEquals(Request.STOP, io.getLastRequest());
    assertFalse(elevator.atGoal());
  }

  /** A switch press sets the position to 0 and homes, once per press. */
  @Test
  public void switchPressHomesOnce() {
    io.setPositionMeters(0.3); // the encoder is wrong at boot
    io.setBottomLimitPressed(true);
    elevator.periodic();
    assertTrue(elevator.isHomed());

    // The reset shows up in the next loop's reading, like on the real motor.
    elevator.periodic();
    assertEquals(0.0, elevator.getPositionMeters(), DELTA);

    // Held down for more loops: still only one reset.
    elevator.periodic();
    elevator.periodic();
    assertEquals(1, io.getPositionResets());

    // Released and pressed again: a second reset.
    io.setBottomLimitPressed(false);
    elevator.periodic();
    io.setBottomLimitPressed(true);
    elevator.periodic();
    assertEquals(2, io.getPositionResets());
  }

  /** Homing drives down at -1.5 V until the switch closes, then holds the bottom. */
  @Test
  public void homingDrivesDownUntilSwitch() {
    io.setPositionMeters(0.4); // boots somewhere up the travel
    elevator.startHoming();
    for (int i = 0; i < 10; i++) {
      elevator.periodic();
      assertEquals(Request.VOLTAGE, io.getLastRequest());
      assertEquals(-1.5, io.getVoltageVolts(), DELTA);
    }
    assertTrue(elevator.isHoming());

    io.setBottomLimitPressed(true);
    elevator.periodic();

    assertTrue(elevator.isHomed());
    assertFalse(elevator.isHoming());
    assertEquals(Request.STOP, io.getLastRequest());
    assertEquals(0.0, elevator.getGoalMeters(), DELTA);
  }

  /** No switch after 200 loops (4 s): homing gives up, stops, and turns on the alert. */
  @Test
  public void homingGivesUpAfterTimeout() {
    elevator.startHoming();
    for (int i = 0; i < 199; i++) {
      elevator.periodic();
    }
    assertEquals(Request.VOLTAGE, io.getLastRequest());
    assertFalse(elevator.homingFailedAlert.get());

    elevator.periodic(); // loop 200
    assertEquals(Request.STOP, io.getLastRequest());
    assertTrue(elevator.homingFailedAlert.get());
    assertTrue(elevator.homingFailed());
    assertFalse(elevator.isHoming());

    // Trying again clears the alert.
    elevator.startHoming();
    elevator.periodic();
    assertFalse(elevator.homingFailedAlert.get());
    assertEquals(Request.VOLTAGE, io.getLastRequest());
  }

  /** Goals are limited to 0 to 1.20 m. */
  @Test
  public void goalIsClampedToTravel() {
    homeAtBottom();

    elevator.setGoal(99);
    elevator.periodic();
    assertEquals(1.20, io.getPositionSetpointMeters(), DELTA);

    elevator.setGoal(-1);
    assertEquals(0.0, elevator.getGoalMeters(), DELTA);
  }

  /** A goal above the bottom goes to the motor's Motion Magic. */
  @Test
  public void goalIsSentToMotor() {
    homeAtBottom();

    elevator.setGoal(0.5);
    elevator.periodic();

    assertEquals(Request.POSITION, io.getLastRequest());
    assertEquals(0.5, io.getPositionSetpointMeters(), DELTA);
  }

  /** To the bottom, Motion Magic to 0.03 m, a gentle -0.5 V push, then no power. */
  @Test
  public void landsGentlyAtTheBottom() {
    homeAtBottom();
    io.setPositionMeters(0.8);

    elevator.setGoal(0.0);
    elevator.periodic();
    assertEquals(Request.POSITION, io.getLastRequest());
    assertEquals(0.03, io.getPositionSetpointMeters(), DELTA);

    io.setPositionMeters(0.03);
    elevator.periodic();
    assertEquals(Request.VOLTAGE, io.getLastRequest());
    assertEquals(-0.5, io.getVoltageVolts(), DELTA);

    io.setBottomLimitPressed(true);
    elevator.periodic();
    assertEquals(Request.STOP, io.getLastRequest());
    // The switch re-zeroes the position; the next loop reads 0, which is at the goal.
    elevator.periodic();
    assertTrue(elevator.atGoal());

    // Resting on the stop must not stop it leaving when asked.
    elevator.setGoal(0.5);
    elevator.periodic();
    assertEquals(Request.POSITION, io.getLastRequest());
  }

  /** Voltage passes through, but stops when driving down onto a pressed switch. */
  @Test
  public void voltageStopsAtBottomSwitch() {
    elevator.setVoltage(-2);
    elevator.periodic();
    assertEquals(Request.VOLTAGE, io.getLastRequest());
    assertEquals(-2, io.getVoltageVolts(), DELTA);

    io.setBottomLimitPressed(true);
    elevator.periodic();
    assertEquals(Request.STOP, io.getLastRequest());

    // Driving up off the switch is allowed.
    elevator.setVoltage(2);
    elevator.periodic();
    assertEquals(Request.VOLTAGE, io.getLastRequest());
    assertEquals(2, io.getVoltageVolts(), DELTA);
  }

  /** Disabling forgets the goal, and it stays stopped after re-enabling. */
  @Test
  public void disablingForgetsTheGoal() {
    homeAtBottom();
    elevator.setGoal(0.5);

    setEnabled(false);
    elevator.periodic();
    assertEquals(Request.STOP, io.getLastRequest());

    setEnabled(true);
    elevator.periodic();
    assertEquals(Request.STOP, io.getLastRequest());
  }

  /** The atGoal check needs homed, a position goal, and within 0.01 m. */
  @Test
  public void atGoalNeedsHomedAndClose() {
    homeAtBottom();
    elevator.setGoal(0.5);

    io.setPositionMeters(0.48);
    elevator.periodic();
    assertFalse(elevator.atGoal());

    io.setPositionMeters(0.495);
    elevator.periodic();
    assertTrue(elevator.atGoal());

    elevator.stop();
    elevator.periodic();
    assertFalse(elevator.atGoal());
  }

  /** Unplugging the motor turns on the alert, and plugging it back turns it off. */
  @Test
  public void disconnectedMotorTurnsOnAlert() {
    elevator.periodic();
    assertFalse(elevator.disconnectedAlert.get());

    io.setConnected(false);
    elevator.periodic();
    assertTrue(elevator.disconnectedAlert.get());

    io.setConnected(true);
    elevator.periodic();
    assertFalse(elevator.disconnectedAlert.get());
  }

  /** Told to move but not at the goal after 100 loops (2 s): "not arrived" alert. */
  @Test
  public void notArrivedAlertAfterTimeout() {
    homeAtBottom();
    elevator.setGoal(0.5);
    for (int i = 0; i < 99; i++) {
      elevator.periodic();
    }
    assertFalse(elevator.notArrivedAlert.get());

    elevator.periodic(); // loop 100
    assertTrue(elevator.notArrivedAlert.get());

    // A new goal starts a fresh 100 loops, even while the alert is on.
    elevator.setGoal(0.8);
    for (int i = 0; i < 99; i++) {
      elevator.periodic();
      assertFalse(elevator.notArrivedAlert.get());
    }

    // Arriving turns it off.
    elevator.periodic(); // loop 100 for the new goal
    assertTrue(elevator.notArrivedAlert.get());
    io.setPositionMeters(0.8);
    elevator.periodic();
    assertFalse(elevator.notArrivedAlert.get());
  }

  /** Dashboard gains reach the motor once per change, not every loop. */
  @Test
  public void gainsSentOnlyWhenChanged() {
    elevator.periodic();
    elevator.periodic();
    assertEquals(0, io.getGainUpdates());

    setDashboardKp(70);
    elevator.periodic();
    elevator.periodic();
    assertEquals(1, io.getGainUpdates());
    assertEquals(70, io.getLastKp(), DELTA);

    // Put the default back, because the dashboard is shared between tests.
    setDashboardKp(60);
    elevator.periodic();
    assertEquals(2, io.getGainUpdates());
    assertEquals(60, io.getLastKp(), DELTA);
  }

  /** Types kP on the pretend dashboard. */
  private void setDashboardKp(double kP) {
    elevator.gains.kP.set(kP);
    elevator.gains.kP.periodic();
  }
}
