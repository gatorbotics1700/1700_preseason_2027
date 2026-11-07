package frc.robot.subsystems.mech.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean flywheelConnected = false;
    public boolean kickerConnected = false;
    public double flywheelRotorRps = 0.0;
    public double flywheelPositionRot = 0.0;
    public double flywheelVolts = 0.0;
    public double flywheelAmps = 0.0;
    public double kickerVolts = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVelocity(double rotorRps) {}

  default void stopFlywheel() {}

  default void setFlywheelVolts(double volts) {}

  default void setFeederSpeeds(double kicker, double left, double right) {}

  default void setFlywheelGains(double kP, double kI, double kD) {}
}
