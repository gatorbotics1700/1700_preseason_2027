package frc.robot.util;

/**
 * Counts robot loops (20 ms each) while something stays true, for timeouts and "told to move but
 * not there" alerts. Counting loops instead of reading the clock keeps tests and replay the same.
 */
public class LoopTimer {

  private final int limitLoops;
  private int loops = 0;

  /** A timer that is done after this many loops in a row. */
  public LoopTimer(int limitLoops) {
    this.limitLoops = limitLoops;
  }

  /** Counts one loop while the condition is true, starts over when it is false. True when done. */
  public boolean update(boolean condition) {
    if (condition) {
      loops++;
    } else {
      loops = 0;
    }
    return loops >= limitLoops;
  }

  /** Starts the count over. */
  public void reset() {
    loops = 0;
  }
}
