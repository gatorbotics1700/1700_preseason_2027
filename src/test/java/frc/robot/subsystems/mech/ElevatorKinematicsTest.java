package frc.robot.subsystems.mech;

import static frc.robot.Constants.ElevatorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ElevatorKinematicsTest {
  private static final double EPS = 1e-9;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
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
}
