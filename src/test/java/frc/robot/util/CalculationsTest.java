package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Tests for Calculations.withinDeadband, which the intake uses to decide "is the arm close enough
 * to its target to stop?". The numbers match the intake: target 95 degrees, deadband 3 degrees.
 */
public class CalculationsTest {

  private static final double TARGET_DEGREES = 95;
  private static final double DEADBAND_DEGREES = 3;

  /** A value well inside the deadband counts as close enough. */
  @Test
  public void insideDeadbandIsClose() {
    assertTrue(Calculations.withinDeadband(TARGET_DEGREES, 94, DEADBAND_DEGREES, true));
    assertTrue(Calculations.withinDeadband(TARGET_DEGREES, 94, DEADBAND_DEGREES, false));
  }

  /** A value outside the deadband is not close enough. */
  @Test
  public void outsideDeadbandIsNotClose() {
    assertFalse(Calculations.withinDeadband(TARGET_DEGREES, 91, DEADBAND_DEGREES, true));
    assertFalse(Calculations.withinDeadband(TARGET_DEGREES, 91, DEADBAND_DEGREES, false));
  }

  /** Exactly on the edge: "true" means the edge counts (<=), "false" means it does not (<). */
  @Test
  public void exactlyOnEdgeDependsOnContainsEquals() {
    assertTrue(Calculations.withinDeadband(TARGET_DEGREES, 92, DEADBAND_DEGREES, true));
    assertFalse(Calculations.withinDeadband(TARGET_DEGREES, 92, DEADBAND_DEGREES, false));
  }

  /** Being below or above the target by the same amount gives the same answer. */
  @Test
  public void directionDoesNotMatter() {
    assertTrue(Calculations.withinDeadband(TARGET_DEGREES, 97, DEADBAND_DEGREES, true));
    assertTrue(Calculations.withinDeadband(97, TARGET_DEGREES, DEADBAND_DEGREES, true));
    assertFalse(Calculations.withinDeadband(TARGET_DEGREES, 99, DEADBAND_DEGREES, true));
  }
}
