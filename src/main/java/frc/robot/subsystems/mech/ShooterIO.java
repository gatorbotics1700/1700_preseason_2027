package frc.robot.subsystems.mech;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {

  public static class ShooterIOInputs implements LoggableInputs {
    public boolean flywheelConnected = false;
    public boolean kickerConnected = false;
    public double flywheelRotorRps = 0.0;
    public double flywheelPositionRot = 0.0; 
    public double flywheelVolts = 0.0;
    public double flywheelAmps = 0.0;
    public double kickerVolts = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("FlywheelConnected", flywheelConnected);
      table.put("KickerConnected", kickerConnected);
      table.put("FlywheelRotorRps", flywheelRotorRps);
      table.put("FlywheelPositionRot", flywheelPositionRot);
      table.put("FlywheelVolts", flywheelVolts);
      table.put("FlywheelAmps", flywheelAmps);
      table.put("KickerVolts", kickerVolts);
    }

    @Override
    public void fromLog(LogTable table) {
      flywheelConnected = table.get("FlywheelConnected", flywheelConnected);
      kickerConnected = table.get("KickerConnected", kickerConnected);
      flywheelRotorRps = table.get("FlywheelRotorRps", flywheelRotorRps);
      flywheelPositionRot = table.get("FlywheelPositionRot", flywheelPositionRot);
      flywheelVolts = table.get("FlywheelVolts", flywheelVolts);
      flywheelAmps = table.get("FlywheelAmps", flywheelAmps);
      kickerVolts = table.get("KickerVolts", kickerVolts);
    }
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVelocity(double rotorRps) {}

  default void stopFlywheel() {}

  default void setFlywheelVolts(double volts) {}

  default void setFeederSpeeds(double kicker, double left, double right) {}

  default void setFlywheelGains(double kP, double kI, double kD) {}
}