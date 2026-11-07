package frc.robot.subsystems.mech.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConditions;
import frc.robot.util.Calculations;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // helpful alerts if motors aren't connected !
  private final Alert flywheelDisconnectedAlert =
      new Alert("Shooter flywheel motor disconnected.", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Shooter kicker motor disconnected.", AlertType.kError);

  private double desiredTransitionSpeed;
  private double desiredRotorVelocity = 0;

  private boolean sysIdRunning = false;
  private SysIdRoutine sysIdRoutine;

  public static LoggedNetworkNumber flyWheelSlip =
      new LoggedNetworkNumber("/Tuning/Shooter/flywheelSlip", 0.255);

  // Tunable PID gains flywheel
  public static final LoggedNetworkNumber flywheelKP =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kP", 0.02);
  public static final LoggedNetworkNumber flywheelKI =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kI", 0.0);
  public static final LoggedNetworkNumber flywheelKD =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kD", 0.002);

  private double lastKP = flywheelKP.get();
  private double lastKI = flywheelKI.get();
  private double lastKD = flywheelKD.get();

  public ShooterSubsystem() {
    this(new ShooterIOTalonFX());
  }

  public ShooterSubsystem(ShooterIO io) {
    // wow very clean
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mech/Shooter", inputs);

    updateSlot0Configs();

    if (!sysIdRunning) {
      if (desiredRotorVelocity == 0.0) { // stops if desired velocity is 0
        io.stopFlywheel();
      } else { // sets speed to desired velocity
        io.setFlywheelVelocity(desiredRotorVelocity);
      }
    }

    io.setFeederSpeeds(
        desiredTransitionSpeed * 2.0,
        desiredTransitionSpeed * 1.5, // TODO
        desiredTransitionSpeed * 0.9);

    flywheelDisconnectedAlert.set(!inputs.flywheelConnected);
    kickerDisconnectedAlert.set(!inputs.kickerConnected);

    shooterLogs();
  }

  public void setDesiredRotorVelocity(double desiredRotorVelocity) {
    this.desiredRotorVelocity = desiredRotorVelocity;
  }

  public double getFlywheelRotorVelocity() {
    return inputs.flywheelRotorRps;
  }

  public void setDesiredTransitionSpeed(double desiredTransitionSpeed) {
    this.desiredTransitionSpeed = desiredTransitionSpeed;
  }

  public double getExitVelocity() {
    return flyWheelSlip.get()
        * getFlywheelRotorVelocity()
        * 2
        * Math.PI
        * ShooterConstants.FLYWHEEL_RADIUS_METERS
        * ShooterConstants.FLYWHEEL_GEAR_RATIO;
  }

  public static double launchSpeedToRotorSpeed(double shotSpeed) {
    return shotSpeed
        / ShooterConstants.FLYWHEEL_GEAR_RATIO
        / 2
        / Math.PI
        / ShooterConstants.FLYWHEEL_RADIUS_METERS
        / flyWheelSlip.get();
  }

  private void initSysIdRoutine() {
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(14),
            Seconds.of(15),
            (state) -> Logger.recordOutput("Mech/Shooter/SysID/SysIdState", state.toString()));

    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              sysIdRunning = true;
              io.setFlywheelVolts(voltage.in(Volts));
            },
            null,
            this,
            "shooter");
    sysIdRoutine = new SysIdRoutine(config, mechanism);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .quasistatic(direction)
        .finallyDo(() -> sysIdRunning = false)
        .withName("Flywheel SysId Quasistatic " + direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .dynamic(direction)
        .finallyDo(() -> sysIdRunning = false)
        .withName("Flywheel SysId Dynamic " + direction);
  }

  public void shooterLogs() {
    Logger.recordOutput(
        "Mech/Shooter/Flywheel/Flywheel Rotor Velocity", getFlywheelRotorVelocity());
    Logger.recordOutput("Mech/Shooter/Flywheel/Desired Rotor Velocity", desiredRotorVelocity);

    Logger.recordOutput("Mech/Shooter/Desired Transition Speed", desiredTransitionSpeed);
    Logger.recordOutput("Mech/Shooter/Kicker/Kicker Running", inputs.kickerVolts != 0);

    Logger.recordOutput("Mech/Shooter/MAX SPEED", ShotCalculatorConditions.MAX_SHOT_SPEED);
    Logger.recordOutput(
        "Mech/Shooter/MAX ROTOR SPEED",
        launchSpeedToRotorSpeed(ShotCalculatorConditions.MAX_SHOT_SPEED));

    // SysID
    Logger.recordOutput("Mech/Shooter/SysID/shooterSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput("Mech/Shooter/SysID/leftShooterVoltage", inputs.flywheelVolts);
      Logger.recordOutput("Mech/Shooter/SysID/leftShooterPosition", inputs.flywheelPositionRot);
      Logger.recordOutput("Mech/Shooter/SysID/leftShooterVelocity", inputs.flywheelRotorRps);
    }
  }

  public void updateSlot0Configs() {
    double kP = flywheelKP.get();
    double kI = flywheelKI.get();
    double kD = flywheelKD.get();
    if (kP != lastKP || kI != lastKI || kD != lastKD) {
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
      io.setFlywheelGains(kP, kI, kD);
    }
  }

  public void runFarthestShot() {
    setDesiredRotorVelocity(90);
    if (Calculations.withinDeadband(
        getFlywheelRotorVelocity(), 90.0, ShooterConstants.FLYWHEEL_SPEED_DEADBAND, false)) {
      setDesiredTransitionSpeed(ShooterConstants.TRANSITION_SPEED);
    }
  }
}
