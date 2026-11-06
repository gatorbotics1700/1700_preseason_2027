package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorKinematicsTest {
  private static final double EPS = 1e-9;

  /** Records every call so tests can assert what the subsystem told the hardware to do. */
  static class FakeIO implements ElevatorIO {
    double position = 0;
    boolean limit = false;
    final List<String> calls = new ArrayList<>();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
      inputs.motorConnected = true;
      inputs.positionMeters = position;
      inputs.bottomLimitPressed = limit;
    }

    @Override
    public void setPositionSetpoint(double positionMeters) {
      calls.add("position " + positionMeters);
    }

    @Override
    public void setVoltage(double volts) {
      calls.add("voltage " + volts);
    }

    @Override
    public void stop() {
      calls.add("stop");
    }

    @Override
    public void resetPosition(double positionMeters) {
      position = positionMeters;
      calls.add("reset " + positionMeters);
    }

    String last() {
      return calls.get(calls.size() - 1);
    }
  }

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void enable() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  // ---- kinematics -----------------------------------------------------------------------

  @Test
  void componentPosesFollowCascadeRigging() {
    for (double h : new double[] {0.0, 0.37, MAX_HEIGHT_METERS}) {
      Pose3d[] poses = ElevatorSubsystem.componentPoses(h);
      assertEquals(2, poses.length);
      assertEquals(h / RIGGING_RATIO, poses[0].getZ(), EPS, "stage 1");
      assertEquals(h, poses[1].getZ(), EPS, "carriage");
      for (Pose3d p : poses) {
        assertEquals(0, p.getX(), EPS);
        assertEquals(0, p.getY(), EPS);
        assertEquals(0, p.getRotation().getAngle(), EPS);
      }
    }
  }

  @Test
  void clampKeepsGoalsInsideTravel() {
    assertEquals(MIN_HEIGHT_METERS, ElevatorSubsystem.clampHeight(-1), EPS);
    assertEquals(MAX_HEIGHT_METERS, ElevatorSubsystem.clampHeight(99), EPS);
    assertEquals(0.5, ElevatorSubsystem.clampHeight(0.5), EPS);
  }

  @Test
  void rotorRatioMatchesDrivetrain() {
    double carriagePerRotor = Math.PI * DRUM_PITCH_DIAMETER_METERS * RIGGING_RATIO / GEAR_RATIO;
    assertEquals(1.0 / carriagePerRotor, ROTOR_ROTATIONS_PER_METER, 1e-9);
  }

  // ---- geometry: the mechanism must physically fit through its full travel ---------------

  @Test
  void setpointsAreInsideTravel() {
    for (double s :
        new double[] {
          STOW_HEIGHT_METERS, LOW_HEIGHT_METERS, MID_HEIGHT_METERS, HIGH_HEIGHT_METERS
        }) {
      assertTrue(s >= MIN_HEIGHT_METERS && s <= MAX_HEIGHT_METERS, "setpoint " + s);
    }
  }

  @Test
  void stagesStayEngagedAtFullExtension() {
    double stage1Travel = MAX_HEIGHT_METERS / RIGGING_RATIO;
    double baseTop = BASE_BOTTOM_Z_METERS + BASE_HEIGHT_METERS;
    double stage1BottomAtMax = STAGE_1_BOTTOM_Z_METERS + stage1Travel;
    double overlap = baseTop - stage1BottomAtMax;
    assertTrue(overlap >= 0.15, "stage 1 overlap with base at max is only " + overlap + " m");
    assertTrue(STAGE_1_BOTTOM_Z_METERS >= BASE_BOTTOM_Z_METERS, "stage 1 below base");
  }

  @Test
  void carriageStaysInsideStage1AtFullExtension() {
    // relative to stage 1, the carriage travels (R - 1) / R of its absolute travel
    double relativeTravel = MAX_HEIGHT_METERS - MAX_HEIGHT_METERS / RIGGING_RATIO;
    double carriageTopRelative =
        CARRIAGE_BOTTOM_Z_METERS
            - STAGE_1_BOTTOM_Z_METERS
            + relativeTravel
            + CARRIAGE_HEIGHT_METERS;
    assertTrue(
        carriageTopRelative <= STAGE_1_HEIGHT_METERS + EPS,
        "carriage overruns stage 1 by " + (carriageTopRelative - STAGE_1_HEIGHT_METERS) + " m");
    assertTrue(CARRIAGE_BOTTOM_Z_METERS >= STAGE_1_BOTTOM_Z_METERS, "carriage below stage 1");
  }

  @Test
  void stagesNestInsideEachOther() {
    assertTrue(STAGE_1_RAIL_SPACING_METERS < BASE_RAIL_SPACING_METERS);
    assertTrue(CARRIAGE_WIDTH_METERS < STAGE_1_RAIL_SPACING_METERS);
  }

  // ---- subsystem safety logic -------------------------------------------------------------

  @Test
  void refusesPositionControlUntilHomed() {
    FakeIO io = new FakeIO();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    elevator.setGoal(0.5);
    elevator.periodic();
    assertFalse(elevator.isHomed());
    assertEquals("stop", io.last());
    assertFalse(elevator.atGoal());
  }

  @Test
  void limitSwitchHomesAndZeroes() {
    FakeIO io = new FakeIO();
    io.position = 0.3; // encoder is wrong at boot
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    io.limit = true;
    elevator.periodic();
    assertTrue(elevator.isHomed());
    assertTrue(io.calls.contains("reset " + MIN_HEIGHT_METERS));

    elevator.setGoal(99);
    io.limit = false;
    elevator.periodic();
    assertEquals("position " + MAX_HEIGHT_METERS, io.last(), "goal must be clamped");
  }

  @Test
  void openLoopStopsAtBottomLimit() {
    FakeIO io = new FakeIO();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    elevator.setVoltage(-2);
    elevator.periodic();
    assertEquals("voltage -2.0", io.last());
    io.limit = true;
    elevator.periodic();
    assertEquals("stop", io.last());
    elevator.setVoltage(2); // driving up off the switch is allowed
    elevator.periodic();
    assertEquals("voltage 2.0", io.last());
  }

  @Test
  void atGoalNeedsToleranceAndHoming() {
    FakeIO io = new FakeIO();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    io.limit = true;
    elevator.periodic();
    elevator.setGoal(0.5);
    io.limit = false;
    io.position = 0.5 - 2 * POSITION_TOLERANCE_METERS;
    elevator.periodic();
    assertFalse(elevator.atGoal());
    io.position = 0.5 - POSITION_TOLERANCE_METERS / 2;
    elevator.periodic();
    assertTrue(elevator.atGoal());
  }

  @Test
  void restsOnHardStopInsteadOfHoldingAtBottom() {
    FakeIO io = new FakeIO();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    io.limit = true;
    elevator.periodic();
    elevator.setGoal(STOW_HEIGHT_METERS);
    elevator.periodic();
    assertEquals("stop", io.last(), "should go neutral on the hard stop");
    assertTrue(elevator.atGoal());
    elevator.setGoal(0.5);
    elevator.periodic();
    assertEquals("position 0.5", io.last(), "must still leave the bottom when asked");
  }

  @Test
  void disablingDropsTheGoal() {
    FakeIO io = new FakeIO();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    io.limit = true;
    elevator.periodic();
    elevator.setGoal(0.5);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    elevator.periodic();
    assertEquals(ElevatorSubsystem.ControlMode.IDLE, elevator.getControlMode());
    assertEquals("stop", io.last());
  }
}
