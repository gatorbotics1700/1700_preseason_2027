package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** LoopTimer is done after the limit in a row, and starts over when told or when false. */
public class LoopTimerTest {

  /** Done on the 3rd loop in a row, not before. */
  @Test
  public void doneAfterLimitInARow() {
    LoopTimer timer = new LoopTimer(3);
    assertFalse(timer.update(true));
    assertFalse(timer.update(true));
    assertTrue(timer.update(true));
  }

  /** A false loop starts the count over. */
  @Test
  public void falseStartsOver() {
    LoopTimer timer = new LoopTimer(3);
    timer.update(true);
    timer.update(true);
    timer.update(false);
    assertFalse(timer.update(true));
    assertFalse(timer.update(true));
    assertTrue(timer.update(true));
  }

  /** reset() starts the count over. */
  @Test
  public void resetStartsOver() {
    LoopTimer timer = new LoopTimer(2);
    timer.update(true);
    timer.reset();
    assertFalse(timer.update(true));
    assertTrue(timer.update(true));
  }
}
