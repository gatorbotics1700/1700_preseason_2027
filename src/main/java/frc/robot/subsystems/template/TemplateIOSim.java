package frc.robot.subsystems.template;

/**
 * TEMPLATE: pretend hardware for tests and the simulator. It has no motor: it just remembers what
 * the subsystem told it, so a test can check it right away, with no waiting.
 *
 * <p>Nothing moves by itself here, so speed, voltage and current read 0. That is expected until
 * someone adds a physics model (see "Simulator" in mechanism-checklist.md).
 */
public class TemplateIOSim implements TemplateIO {

  private double speed = 0.0;
  // Starts connected. A test can unplug it to check the alert.
  private boolean connected = true;

  /** Reports whatever the test has set up. */
  @Override
  public void updateInputs(TemplateIOInputs inputs) {
    inputs.motorConnected = connected;
    inputs.motorVelocityRps = 0.0;
    inputs.motorVoltageVolts = 0.0;
    inputs.motorCurrentAmps = 0.0;
  }

  /** Remembers the speed. */
  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  // ---------------- For tests ----------------

  /** The last speed the subsystem asked for. */
  public double getSpeed() {
    return speed;
  }

  /** Plugs in or unplugs the pretend motor. */
  public void setConnected(boolean connected) {
    this.connected = connected;
  }
}
