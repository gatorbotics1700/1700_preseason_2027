package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.logging.TalonFXLogger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX deployMotor;

  private final DigitalInput retractedLimitSwitch;
  private final DigitalInput deployedLimitSwitch;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 3;
  private boolean sysIdRunning = false;

  private final TalonFXConfiguration deployTalonFXConfigs;
  private final TalonFXConfiguration intakeTalonFXConfigs;

  private static CurrentLimitsConfigs deployCurrentLimitConfigs;
  private static CurrentLimitsConfigs intakeCurrentLimitConfigs;

  public static final LoggedNetworkNumber intakeCurrentLimit =
      new LoggedNetworkNumber("/Tuning/Intake/Intake Current Limit", 25);

  /** When true, drive toward deployed sensor; when false, toward retract sensor. */
  private boolean deployGoalExtended = false;

  /**
   * When true, {@link #deployManualSpeed} is applied every tick. When false, goal and halls pick
   * speed.
   */
  private boolean deployManualControl = false;

  private double deployManualSpeed = 0.0;

  private double desiredIntakeSpeed;

  private boolean prevRetractSwitch = false;
  private boolean prevDeployedSwitch = false;
  private boolean wasSeekingRetractSwitch = false;
  private boolean wasSeekingDeploySwitch = false;

  public void setSysIdRunning(boolean running) {
    sysIdRunning = running;
  }

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    deployMotor =
        new TalonFX(IntakeConstants.INTAKE_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredIntakeSpeed = 0;
    retractedLimitSwitch = new DigitalInput(IntakeConstants.INTAKE_LIMIT_SWITCH_PORT);
    deployedLimitSwitch = new DigitalInput(IntakeConstants.DEPLOYED_LIMIT_SWITCH_PORT);

    intakeTalonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    deployTalonFXConfigs = new TalonFXConfiguration();

    deployTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    intakeCurrentLimitConfigs = intakeTalonFXConfigs.CurrentLimits;
    intakeCurrentLimitConfigs.StatorCurrentLimit = 25;
    intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    deployCurrentLimitConfigs = deployTalonFXConfigs.CurrentLimits;
    deployCurrentLimitConfigs.StatorCurrentLimit = 35;
    deployCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    intakeMotor.getConfigurator().apply(intakeTalonFXConfigs);

    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    updateCurrentLimitConfigs();
    // updateDeployStatorLimitForPosition();

    if (!sysIdRunning) {
      if (deployManualControl) {
        runManualDeployControl();
      } else {
        runGoalBasedDeployControl();
      }
    }

    intakeMotor.set(desiredIntakeSpeed);
    intakeLogs();
  }

  /**
   * Manual mode directly applies commanded deploy speed. Sensor transitions are still watched so we
   * can re-zero encoder position at the physical endpoints.
   */
  private void runManualDeployControl() {
    deployMotor.set(deployManualSpeed);

    boolean retractSwitch = isRetractedLimitSwitchTriggered();
    boolean deployedSwitch = isDeployedHallEffectTriggered();
    boolean seekingRetract = deployManualSpeed > 0;
    boolean seekingDeploy = deployManualSpeed < 0;

    if (seekingRetract && !prevRetractSwitch && retractSwitch) {
      rehomeToRetractedStop();
    }
    if (seekingDeploy && !prevDeployedSwitch && deployedSwitch) {
      rehomeToDeployedStop();
    }

    prevRetractSwitch = retractSwitch;
    prevDeployedSwitch = deployedSwitch;
    wasSeekingRetractSwitch = seekingRetract;
    wasSeekingDeploySwitch = seekingDeploy;
  }

  /**
   * Automatic mode drives toward one of the two hall-defined endpoints until it trips, then stops.
   */
  private void runGoalBasedDeployControl() {
    boolean retractHall = isRetractedLimitSwitchTriggered();
    boolean deployedHall = isDeployedHallEffectTriggered();

    applyDeployGoalMotorOutput(retractHall, deployedHall);
    handleGoalBasedRehomeTransitions(retractHall, deployedHall);

    prevRetractSwitch = retractHall;
    prevDeployedSwitch = deployedHall;
  }

  private void applyDeployGoalMotorOutput(boolean retractHall, boolean deployedHall) {
    if (deployGoalExtended && !deployedHall) {
      deployMotor.set(-IntakeConstants.HOMING_SPEED);
      wasSeekingDeploySwitch = true;
      wasSeekingRetractSwitch = false;
    } else if (!deployGoalExtended && !retractHall) {
      deployMotor.set(IntakeConstants.HOMING_SPEED);
      wasSeekingRetractSwitch = true;
      wasSeekingDeploySwitch = false;
    } else {
      deployMotor.set(0);
    }
  }

  private void handleGoalBasedRehomeTransitions(boolean retractHall, boolean deployedHall) {
    if (!prevRetractSwitch && retractHall && wasSeekingRetractSwitch) {
      rehomeToRetractedStop();
      wasSeekingRetractSwitch = false;
    }
    if (!prevDeployedSwitch && deployedHall && wasSeekingDeploySwitch) {
      rehomeToDeployedStop();
      wasSeekingDeploySwitch = false;
    }
  }

  private void rehomeToRetractedStop() {
    zeroIntakeDeploy(true);
    Logger.recordOutput("Mech/Intake/Deploy/RehomeFromHall", "retract");
  }

  private void rehomeToDeployedStop() {
    zeroIntakeDeploy(false);
    Logger.recordOutput("Mech/Intake/Deploy/RehomeFromHall", "deploy");
  }

  private void updateDeployStatorLimitForPosition() {
    if (isDeployedHallEffectTriggered() && deployCurrentLimitConfigs.StatorCurrentLimit != 35) {
      deployCurrentLimitConfigs.StatorCurrentLimit = 35;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
    if (!isDeployedHallEffectTriggered() && deployCurrentLimitConfigs.StatorCurrentLimit != 60) {
      deployCurrentLimitConfigs.StatorCurrentLimit = 60;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
  }

  /** Open-loop deploy speed until {@link #clearDeployManualControl()}. */
  public void setDeploySpeed(double speed) {
    deployManualControl = true;
    deployManualSpeed = speed;
  }

  public void clearDeployManualControl() {
    deployManualControl = false;
    deployManualSpeed = 0;
  }

  public void setDeployGoalExtended(boolean extended) {
    deployGoalExtended = extended;
    deployManualControl = false;
  }

  public boolean getDeployGoalExtended() {
    return deployGoalExtended;
  }

  public void setIntakeSpeed(double speed) {
    desiredIntakeSpeed = speed;
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = deployMotor.getPosition().getValueAsDouble();
    double deployAngleDegrees =
        motorPositionRevs
            / IntakeConstants.DEPLOY_GEARBOX_RATIO
            / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
            / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
            * 360.0
            % 360;
    return new Rotation2d(Math.toRadians(deployAngleDegrees));
  }

  public double degreesToRevs(double deployAngleDegrees) {
    return deployAngleDegrees
        / 360.0
        * IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        * IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        * IntakeConstants.DEPLOY_GEARBOX_RATIO;
  }

  public void zeroIntakeDeploy(boolean isRetracted) {
    if (isRetracted) {
      deployMotor.setPosition(degreesToRevs(IntakeConstants.RETRACTED_POSITION.getDegrees()));
    } else {
      deployMotor.setPosition(degreesToRevs(IntakeConstants.EXTENDED_POSITION.getDegrees()));
    }
  }

  public boolean isRetractedLimitSwitchTriggered() {
    return !retractedLimitSwitch.get();
  }

  public boolean isDeployedHallEffectTriggered() {
    return !deployedLimitSwitch.get();
  }

  /** True when the deploy goal matches the corresponding hall (at commanded stop). */
  public boolean atDeployGoal() {
    if (deployGoalExtended) {
      return isDeployedHallEffectTriggered();
    }
    return isRetractedLimitSwitchTriggered();
  }

  public void retractDeployMotor() {
    setDeployGoalExtended(false);
  }

  public void extendDeployMotor() {
    setDeployGoalExtended(true);
  }

  /**
   * Maps a nominal angle to deploy extended vs retract for legacy call sites. Midpoint between
   * retract and extended constants is the threshold; deploy motion is still open-loop to halls.
   */
  public void setDesiredAngle(Rotation2d angle) {
    double deg = angle.getDegrees();
    double mid =
        (IntakeConstants.EXTENDED_ANGLE_DEGREES + IntakeConstants.RETRACTED_ANGLE_DEGREES) / 2.0;
    setDeployGoalExtended(deg >= mid);
  }

  public Rotation2d getDesiredAngle() {
    return deployGoalExtended
        ? IntakeConstants.EXTENDED_POSITION
        : IntakeConstants.RETRACTED_POSITION;
  }

  /** True when the deployed hall is active (physical end of travel). */
  public BooleanSupplier getIsDeployed() {
    return this::isDeployedHallEffectTriggered;
  }

  private double getVelocityRadPerSec() {
    double motorRPS = deployMotor.getVelocity().getValueAsDouble();
    return motorRPS
        / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        / IntakeConstants.DEPLOY_GEARBOX_RATIO
        * 2
        * Math.PI;
  }

  private SysIdRoutine sysIdRoutine() {
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            Volts.per(Second).of(2),
            Volts.of(18),
            Seconds.of(10),
            (state) -> Logger.recordOutput("Mech/Intake/SysID/SysIdState", state.toString()));

    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> deployMotor.setVoltage(voltage.in(Volts)), null, this, "intake");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    boolean isSysIdOutOfBounds =
        angleDeg >= IntakeConstants.EXTENDED_ANGLE_DEGREES + SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg <= IntakeConstants.RETRACTED_ANGLE_DEGREES - SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Intake/SysID/Outofbounds", isSysIdOutOfBounds);

    return isSysIdOutOfBounds;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .quasistatic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Intake SysId Quasistatic " + direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .dynamic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Intake SysId Dynamic " + direction);
  }

  public void updateCurrentLimitConfigs() {
    double newIntakeCurrentLimit = intakeCurrentLimit.get();

    if (newIntakeCurrentLimit != intakeCurrentLimitConfigs.StatorCurrentLimit) {
      intakeCurrentLimitConfigs.StatorCurrentLimit = newIntakeCurrentLimit;
      intakeMotor.getConfigurator().apply(intakeTalonFXConfigs);
    }
  }

  public void intakeLogs() {
    TalonFXLogger.log(deployMotor, "Mech", "Intake", "Deploy");
    TalonFXLogger.log(intakeMotor, "Mech", "Intake", "Intake");

    Logger.recordOutput("Mech/Intake/Deploy/Current Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Deploy/Goal Extended", deployGoalExtended);
    Logger.recordOutput("Mech/Intake/Deploy/Manual Control", deployManualControl);
    Logger.recordOutput("Mech/Intake/Deploy/Manual Speed", deployManualSpeed);
    Logger.recordOutput(
        "Mech/Intake/Deploy/Current Limit", deployCurrentLimitConfigs.StatorCurrentLimit);

    Logger.recordOutput("Mech/Intake/Intake Hall Effect", isRetractedLimitSwitchTriggered());
    Logger.recordOutput("Mech/Intake/Deployed Hall Effect", isDeployedHallEffectTriggered());
    Logger.recordOutput("Mech/Intake/IsDeployed", isDeployedHallEffectTriggered());
    Logger.recordOutput("Mech/Intake/Intake/Desired Intake Speed", desiredIntakeSpeed);
    Logger.recordOutput(
        "Mech/Intake/Intake/Current Limit", intakeCurrentLimitConfigs.StatorCurrentLimit);

    Logger.recordOutput("Mech/Intake/SysID/intakeSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVoltage", deployMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Intake/SysID/intakePosition", getCurrentAngle().getRadians() / (2.0 * Math.PI));
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVelocity", getVelocityRadPerSec() / (2.0 * Math.PI));
    }
  }
}
