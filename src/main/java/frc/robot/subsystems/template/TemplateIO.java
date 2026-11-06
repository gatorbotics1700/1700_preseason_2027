package frc.robot.subsystems.template;

import org.littletonrobotics.junction.AutoLog;

/**
 * TEMPLATE: the hardware layer for a mechanism with one motor that runs at a set speed (a roller).
 * Copy this folder to start a new mechanism. See README.md in this folder.
 *
 * <p>TemplateSubsystem makes the decisions and talks to the hardware only through this interface,
 * so a simulated version can stand in for the real motor.
 *
 * <p>An interface is a list of methods with no hardware code of its own. Each version fills them
 * in:
 *
 * <ul>
 *   <li>TemplateIOTalonFX: the real motor.
 *   <li>TemplateIOSim: pretend hardware that remembers what it was told, for tests and simulation.
 *   <li>new TemplateIO() {}: does nothing at all, because every method below has an empty default.
 *       Use it to turn the mechanism off without unplugging anything.
 * </ul>
 */
public interface TemplateIO {

  /** Everything the mechanism reads from its hardware. AdvantageKit logs these every loop. */
  @AutoLog
  public static class TemplateIOInputs {
    public boolean motorConnected = false;
    public double motorVelocityRps = 0.0;
    public double motorVoltageVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    // TODO: add anything else your mechanism reads, such as a sensor. Put units in the names.
  }

  /** Reads the hardware into inputs. Called once at the start of every loop. */
  public default void updateInputs(TemplateIOInputs inputs) {}

  /** Runs the motor at a speed from -1 (full reverse) to 1 (full forward). */
  public default void setSpeed(double speed) {}

  // TODO: add one method for each thing your mechanism tells the hardware to do.
}
