package frc.robot.subsystems.mech;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ElevatorConstants;
import org.junit.jupiter.api.Test;

/** The elevator's gearing and sizes fit together through its full travel. */
public class ElevatorKinematicsTest {
  private static final double DELTA = 1e-9;

  /** Motor rotations per meter match the gearing, sprocket and rigging. */
  @Test
  public void rotorRatioMatchesDrivetrain() {
    double carriagePerRotor =
        Math.PI
            * ElevatorConstants.DRUM_PITCH_DIAMETER_METERS
            * ElevatorConstants.RIGGING_RATIO
            / ElevatorConstants.GEAR_RATIO;
    assertEquals(1.0 / carriagePerRotor, ElevatorConstants.ROTOR_ROTATIONS_PER_METER, 1e-9);
  }

  /** Every named height is inside the travel limits. */
  @Test
  public void setpointsAreInsideTravel() {
    for (double s :
        new double[] {
          ElevatorConstants.STOW_HEIGHT_METERS,
          ElevatorConstants.LOW_HEIGHT_METERS,
          ElevatorConstants.MID_HEIGHT_METERS,
          ElevatorConstants.HIGH_HEIGHT_METERS
        }) {
      assertTrue(
          s >= ElevatorConstants.MIN_HEIGHT_METERS && s <= ElevatorConstants.MAX_HEIGHT_METERS,
          "setpoint " + s);
    }
  }

  /** At full height, stage 1 still overlaps the fixed stage enough to be stiff. */
  @Test
  public void stagesStayEngagedAtFullExtension() {
    double stage1Travel = ElevatorConstants.MAX_HEIGHT_METERS / ElevatorConstants.RIGGING_RATIO;
    double baseTop = ElevatorConstants.BASE_BOTTOM_Z_METERS + ElevatorConstants.BASE_HEIGHT_METERS;
    double stage1BottomAtMax = ElevatorConstants.STAGE_1_BOTTOM_Z_METERS + stage1Travel;
    double overlap = baseTop - stage1BottomAtMax;
    assertTrue(overlap >= 0.15, "stage 1 overlap with base at max is only " + overlap + " m");
    assertTrue(
        ElevatorConstants.STAGE_1_BOTTOM_Z_METERS >= ElevatorConstants.BASE_BOTTOM_Z_METERS,
        "stage 1 below base");
  }

  /** At full height, the carriage still sits inside stage 1. */
  @Test
  public void carriageStaysInsideStage1AtFullExtension() {
    // Relative to stage 1, the carriage travels (R - 1) / R of its absolute travel.
    double relativeTravel =
        ElevatorConstants.MAX_HEIGHT_METERS
            - ElevatorConstants.MAX_HEIGHT_METERS / ElevatorConstants.RIGGING_RATIO;
    double carriageTopRelative =
        ElevatorConstants.CARRIAGE_BOTTOM_Z_METERS
            - ElevatorConstants.STAGE_1_BOTTOM_Z_METERS
            + relativeTravel
            + ElevatorConstants.CARRIAGE_HEIGHT_METERS;
    assertTrue(
        carriageTopRelative <= ElevatorConstants.STAGE_1_HEIGHT_METERS + DELTA,
        "carriage overruns stage 1 by "
            + (carriageTopRelative - ElevatorConstants.STAGE_1_HEIGHT_METERS)
            + " m");
    assertTrue(
        ElevatorConstants.CARRIAGE_BOTTOM_Z_METERS >= ElevatorConstants.STAGE_1_BOTTOM_Z_METERS,
        "carriage below stage 1");
  }

  /** Each stage is narrower than the one it rides in. */
  @Test
  public void stagesNestInsideEachOther() {
    assertTrue(
        ElevatorConstants.STAGE_1_RAIL_SPACING_METERS < ElevatorConstants.BASE_RAIL_SPACING_METERS);
    assertTrue(
        ElevatorConstants.CARRIAGE_WIDTH_METERS < ElevatorConstants.STAGE_1_RAIL_SPACING_METERS);
  }
}
