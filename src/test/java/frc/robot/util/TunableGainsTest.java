package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** TunableGains reports a change once per dashboard edit, not every loop. */
public class TunableGainsTest {

  /** Starts WPILib, so the pretend dashboard works. */
  @BeforeAll
  public static void startWpilib() {
    HAL.initialize(500, 0);
  }

  /** Nothing changed at first; one edit is reported once. */
  @Test
  public void reportsEachChangeOnce() {
    TunableGains gains = new TunableGains("Test", 60, 0, 1, 0.05, 0.8, 2.55, 0.07);
    assertFalse(gains.changed());

    gains.kP.set(70);
    gains.kP.periodic();
    assertTrue(gains.changed());
    assertFalse(gains.changed());
  }
}
