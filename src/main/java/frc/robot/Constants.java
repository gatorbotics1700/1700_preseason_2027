// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Calculations;
import frc.robot.util.RobotConfigLoader;
import frc.robot.util.shooting.ShotParameters;

/**
 * This class defines the runtime mode used by AdvantageKit and loads robot-specific configuration
 * based on the roboRIO serial number. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a
 * file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Robot identification
  public static final String ROBOT_SERIAL_NUMBER;

  static {
    // Load configuration based on roboRIO serial number (auto-loads on first access)
    ROBOT_SERIAL_NUMBER = RobotConfigLoader.getSerialNumber();
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int KRAKEN_TICKS_PER_REV = 2048;

  public static final class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(RobotConfigLoader.getDouble("tuner.steer_kp"))
            .withKI(RobotConfigLoader.getDouble("tuner.steer_ki"))
            .withKD(RobotConfigLoader.getDouble("tuner.steer_kd"))
            .withKS(RobotConfigLoader.getDouble("tuner.steer_ks"))
            .withKV(RobotConfigLoader.getDouble("tuner.steer_kv"))
            .withKA(RobotConfigLoader.getDouble("tuner.steer_ka"))
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(RobotConfigLoader.getDouble("tuner.drive_kp"))
            .withKI(RobotConfigLoader.getDouble("tuner.drive_ki"))
            .withKD(RobotConfigLoader.getDouble("tuner.drive_kd"))
            .withKS(RobotConfigLoader.getDouble("tuner.drive_ks"))
            .withKV(RobotConfigLoader.getDouble("tuner.drive_kv"))
            .withKA(RobotConfigLoader.getDouble("tuner.drive_ka"));

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType =
        DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType =
        SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent =
        Amps.of(RobotConfigLoader.getDouble("tuner.slip_current_amps"));

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(
                        Amps.of(RobotConfigLoader.getDouble("tuner.stator_current_limit_amps")))
                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus driveCANBus =
        new CANBus(
            RobotConfigLoader.getString("tuner.drive_canbus_name").equals("null")
                ? ""
                : RobotConfigLoader.getString("tuner.drive_canbus_name"),
            "U/logs");

    // Mechanism CAN bus - reuses driveCANBus if they're the same physical bus
    public static final CANBus mechCANBus;

    static {
      String driveBusName = driveCANBus.getName();
      String mechBusName =
          RobotConfigLoader.getString("tuner.mech_canbus_name").equals("null")
              ? ""
              : RobotConfigLoader.getString("tuner.mech_canbus_name");

      // If both buses have the same name, reuse the same object
      if (driveBusName.equals(mechBusName)) {
        mechCANBus = driveCANBus;
      } else {
        mechCANBus = new CANBus(mechBusName, "./logs/example.hoot");
      }
    }

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts =
        MetersPerSecond.of(RobotConfigLoader.getDouble("tuner.speed_at_12_volts_meters_per_sec"));

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = RobotConfigLoader.getDouble("tuner.couple_ratio");

    private static final double kDriveGearRatio =
        RobotConfigLoader.getDouble("tuner.drive_gear_ratio");
    private static final double kSteerGearRatio =
        RobotConfigLoader.getDouble("tuner.steer_gear_ratio");
    private static final Distance kWheelRadius =
        Inches.of(RobotConfigLoader.getDouble("tuner.wheel_radius_inches"));

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = RobotConfigLoader.getInt("tuner.pigeon_id");

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(driveCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId =
        RobotConfigLoader.getInt("tuner.front_left_drive_motor_id");
    private static final int kFrontLeftSteerMotorId =
        RobotConfigLoader.getInt("tuner.front_left_steer_motor_id");
    private static final int kFrontLeftEncoderId =
        RobotConfigLoader.getInt("tuner.front_left_encoder_id");
    private static final Angle kFrontLeftEncoderOffset =
        Rotations.of(RobotConfigLoader.getDouble("tuner.front_left_encoder_offset_rotations"));
    private static final boolean kFrontLeftSteerMotorInverted =
        RobotConfigLoader.getBoolean("tuner.front_left_steer_motor_inverted");
    private static final boolean kFrontLeftEncoderInverted =
        RobotConfigLoader.getBoolean("tuner.front_left_steer_encoder_inverted");

    private static final Distance kFrontLeftXPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.front_left_pos.x_inches"));
    private static final Distance kFrontLeftYPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.front_left_pos.y_inches"));

    // Front Right
    private static final int kFrontRightDriveMotorId =
        RobotConfigLoader.getInt("tuner.front_right_drive_motor_id");
    private static final int kFrontRightSteerMotorId =
        RobotConfigLoader.getInt("tuner.front_right_steer_motor_id");
    private static final int kFrontRightEncoderId =
        RobotConfigLoader.getInt("tuner.front_right_encoder_id");
    private static final Angle kFrontRightEncoderOffset =
        Rotations.of(RobotConfigLoader.getDouble("tuner.front_right_encoder_offset_rotations"));
    private static final boolean kFrontRightSteerMotorInverted =
        RobotConfigLoader.getBoolean("tuner.front_right_steer_motor_inverted");
    private static final boolean kFrontRightEncoderInverted =
        RobotConfigLoader.getBoolean("tuner.front_right_steer_encoder_inverted");

    private static final Distance kFrontRightXPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.front_right_pos.x_inches"));
    private static final Distance kFrontRightYPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.front_right_pos.y_inches"));

    // Back Left
    private static final int kBackLeftDriveMotorId =
        RobotConfigLoader.getInt("tuner.back_left_drive_motor_id");
    private static final int kBackLeftSteerMotorId =
        RobotConfigLoader.getInt("tuner.back_left_steer_motor_id");
    private static final int kBackLeftEncoderId =
        RobotConfigLoader.getInt("tuner.back_left_encoder_id");
    private static final Angle kBackLeftEncoderOffset =
        Rotations.of(RobotConfigLoader.getDouble("tuner.back_left_encoder_offset_rotations"));
    private static final boolean kBackLeftSteerMotorInverted =
        RobotConfigLoader.getBoolean("tuner.back_left_steer_motor_inverted");
    private static final boolean kBackLeftEncoderInverted =
        RobotConfigLoader.getBoolean("tuner.back_left_steer_encoder_inverted");

    private static final Distance kBackLeftXPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.back_left_pos.x_inches"));
    private static final Distance kBackLeftYPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.back_left_pos.y_inches"));

    // Back Right
    private static final int kBackRightDriveMotorId =
        RobotConfigLoader.getInt("tuner.back_right_drive_motor_id");
    private static final int kBackRightSteerMotorId =
        RobotConfigLoader.getInt("tuner.back_right_steer_motor_id");
    private static final int kBackRightEncoderId =
        RobotConfigLoader.getInt("tuner.back_right_encoder_id");
    private static final Angle kBackRightEncoderOffset =
        Rotations.of(RobotConfigLoader.getDouble("tuner.back_right_encoder_offset_rotations"));
    private static final boolean kBackRightSteerMotorInverted =
        RobotConfigLoader.getBoolean("tuner.back_right_steer_motor_inverted");
    private static final boolean kBackRightEncoderInverted =
        RobotConfigLoader.getBoolean("tuner.back_right_steer_encoder_inverted");

    private static final Distance kBackRightXPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.back_right_pos.y_inches"));
    private static final Distance kBackRightYPos =
        Inches.of(RobotConfigLoader.getDouble("tuner.back_right_pos.y_inches"));

    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                kFrontLeftXPos,
                kFrontLeftYPos,
                kInvertLeftSide,
                kFrontLeftSteerMotorInverted,
                kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                kFrontRightXPos,
                kFrontRightYPos,
                kInvertRightSide,
                kFrontRightSteerMotorInverted,
                kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                kBackLeftXPos,
                kBackLeftYPos,
                kInvertLeftSide,
                kBackLeftSteerMotorInverted,
                kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                kBackRightXPos,
                kBackRightYPos,
                kInvertRightSide,
                kBackRightSteerMotorInverted,
                kBackRightEncoderInverted);
  }

  public static final class VisionConstants {
    // AprilTag layout
    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static final String CAMERA_0_NAME = RobotConfigLoader.getString("camera.0.name");

    public static final double ROBOT_TO_CAMERA_0_X_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.x_meters");
    public static final double ROBOT_TO_CAMERA_0_Y_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.y_meters");
    public static final double ROBOT_TO_CAMERA_0_Z_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.z_meters");
    public static final double CAMERA_0_ROLL_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.roll_degrees");
    public static final double CAMERA_0_PITCH_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.pitch_degrees");
    public static final double ROBOT_TO_CAMERA_0_YAW_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.yaw_degrees");

    public static Transform3d ROBOT_TO_CAMERA_0 = createRobotToCamera0Transform();

    // Camera names, must match names configured on coprocessor
    public static final String CAMERA_1_NAME = RobotConfigLoader.getString("camera.1.name");

    public static final double ROBOT_TO_CAMERA_1_X_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.x_meters");
    public static final double ROBOT_TO_CAMERA_1_Y_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.y_meters");
    public static final double ROBOT_TO_CAMERA_1_Z_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.z_meters");
    public static final double ROBOT_TO_CAMERA_1_ROLL_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.roll_degrees");
    public static final double ROBOT_TO_CAMERA_1_PITCH_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.pitch_degrees");
    public static final double ROBOT_TO_CAMERA_1_YAW_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.yaw_degrees");

    public static Transform3d ROBOT_TO_CAMERA_1 = createRobotToCamera1Transform();

    public static final String CAMERA_2_NAME = RobotConfigLoader.getString("camera.2.name");

    public static final double ROBOT_TO_CAMERA_2_X_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_2.x_meters");
    public static final double ROBOT_TO_CAMERA_2_Y_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_2.y_meters");
    public static final double ROBOT_TO_CAMERA_2_Z_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_2.z_meters");
    public static final double ROBOT_TO_CAMERA_2_ROLL_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_2.roll_degrees");
    public static final double ROBOT_TO_CAMERA_2_PITCH_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_2.pitch_degrees");
    public static final double ROBOT_TO_CAMERA_2_YAW_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_2.yaw_degrees");

    public static Transform3d ROBOT_TO_CAMERA_2 = createRobotToCamera2Transform();

    public static final String CAMERA_3_NAME = RobotConfigLoader.getString("camera.3.name");
    public static final double ROBOT_TO_CAMERA_3_X_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_3.x_meters");
    public static final double ROBOT_TO_CAMERA_3_Y_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_3.y_meters");
    public static final double ROBOT_TO_CAMERA_3_Z_METERS =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_3.z_meters");
    public static final double ROBOT_TO_CAMERA_3_ROLL_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_3.roll_degrees");
    public static final double ROBOT_TO_CAMERA_3_PITCH_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_3.pitch_degrees");
    public static final double ROBOT_TO_CAMERA_3_YAW_DEGREES =
        RobotConfigLoader.getDouble("photonvision.robot_to_camera_3.yaw_degrees");

    public static Transform3d ROBOT_TO_CAMERA_3 = createRobotToCamera3Transform();

    public static Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS_ARRAY = createCameraTransformsArray();

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = RobotConfigLoader.getDouble("photonvision.max_ambiguity");
    public static double MAX_Z_ERROR = RobotConfigLoader.getDouble("photonvision.max_z_error");

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STD_DEV_BASELINE =
        RobotConfigLoader.getDouble("photonvision.linear_std_dev_baseline"); // Meters
    public static double ANGULAR_STD_DEV_BASELINE =
        RobotConfigLoader.getDouble("photonvision.angular_std_dev_baseline"); // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)

    public static double CAMERA_0_STD_DEV_FACTOR =
        RobotConfigLoader.getDouble("photonvision.camera0_std_dev_factor");

    public static double CAMERA_1_STD_DEV_FACTOR =
        RobotConfigLoader.getDouble("photonvision.camera1_std_dev_factor");

    public static double[] CAMERA_STD_DEV_FACTORS = createCameraStdDevFactors();

    // Multipliers to apply for PhotonVision multitag observations
    public static double LINEAR_STD_DEV_MULTITAG_FACTOR =
        RobotConfigLoader.getDouble(
            "photonvision.linear_std_dev_multitag_factor"); // More stable than single tag
    public static double ANGULAR_STD_DEV_MULTITAG_FACTOR =
        RobotConfigLoader.getDouble("photonvision.angular_std_dev_multitag_factor");

    public static int FUEL_CLASS_ID = 0;

    public static Transform3d createRobotToCamera0Transform() {
      return createRobotToCameraTransform(
          ROBOT_TO_CAMERA_0_X_METERS,
          ROBOT_TO_CAMERA_0_Y_METERS,
          ROBOT_TO_CAMERA_0_Z_METERS,
          CAMERA_0_ROLL_DEGREES,
          CAMERA_0_PITCH_DEGREES,
          ROBOT_TO_CAMERA_0_YAW_DEGREES);
    }

    public static Transform3d createRobotToCamera1Transform() {
      return createRobotToCameraTransform(
          ROBOT_TO_CAMERA_1_X_METERS,
          ROBOT_TO_CAMERA_1_Y_METERS,
          ROBOT_TO_CAMERA_1_Z_METERS,
          ROBOT_TO_CAMERA_1_ROLL_DEGREES,
          ROBOT_TO_CAMERA_1_PITCH_DEGREES,
          ROBOT_TO_CAMERA_1_YAW_DEGREES);
    }

    public static Transform3d createRobotToCamera2Transform() {
      return createRobotToCameraTransform(
          ROBOT_TO_CAMERA_2_X_METERS,
          ROBOT_TO_CAMERA_2_Y_METERS,
          ROBOT_TO_CAMERA_2_Z_METERS,
          ROBOT_TO_CAMERA_2_ROLL_DEGREES,
          ROBOT_TO_CAMERA_2_PITCH_DEGREES,
          ROBOT_TO_CAMERA_2_YAW_DEGREES);
    }

    public static Transform3d createRobotToCamera3Transform() {
      return createRobotToCameraTransform(
          ROBOT_TO_CAMERA_3_X_METERS,
          ROBOT_TO_CAMERA_3_Y_METERS,
          ROBOT_TO_CAMERA_3_Z_METERS,
          ROBOT_TO_CAMERA_3_ROLL_DEGREES,
          ROBOT_TO_CAMERA_3_PITCH_DEGREES,
          ROBOT_TO_CAMERA_3_YAW_DEGREES);
    }

    public static Transform3d createRobotToCameraTransform(
        double xMeters,
        double yMeters,
        double zMeters,
        double rollDegrees,
        double pitchDegrees,
        double yawDegrees) {
      Pose3d rotation =
          new Pose3d(new Translation3d(), new Rotation3d(0, 0, Math.toRadians(yawDegrees)));
      rotation =
          rotation.transformBy(
              new Transform3d(
                  new Translation3d(),
                  new Rotation3d(Math.toRadians(rollDegrees), Math.toRadians(pitchDegrees), 0)));
      return new Transform3d(xMeters, yMeters, zMeters, rotation.getRotation());
    }

    public static Transform3d[] createCameraTransformsArray() {
      Transform3d[] array = {
        ROBOT_TO_CAMERA_0, ROBOT_TO_CAMERA_1, ROBOT_TO_CAMERA_2, ROBOT_TO_CAMERA_3
      };
      return array;
    }

    /** Creates array of camera std dev factors from config values. */
    public static double[]
        createCameraStdDevFactors() { // can add more constants if we have more cameras
      return new double[] {CAMERA_0_STD_DEV_FACTOR, CAMERA_1_STD_DEV_FACTOR};
    }

    public static final double DISTANCE_DEADBAND_METERS = 0.03;
    public static final double ROTATION_DEADBAND_DEGREES = 10;

    public static final double TARGET_ANGLE_SCALAR = 1.28054;
  }

  public static final class DriveToFuelConstants {
    public static final double BLIND_SPOT_DEADBAND = 0.5; // TODO change
    public static final double MAX_IDLE_MILLISECONDS =
        2000; // TODO change based off real world maybe?
    public static final double ROTATING_SPEED_RADIANS_PER_SECOND =
        2.5; // TODO change based off real life
  }

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_CAN_ID = 17;
    public static final int HOOD_LIMIT_SWITCH_PORT = 9;

    // retracted position is the max hood angle, because we measure from vertical
    public static final Rotation2d RETRACTED_POSITION =
        new Rotation2d(Math.toRadians(RobotConfigLoader.getInt("mech.hood_retracted_degrees")));
    public static final Rotation2d MIN_ANGLE =
        new Rotation2d(Math.toRadians(RobotConfigLoader.getInt("mech.hood_min_angle_degrees")));

    public static final double HOOD_POSITION_DEADBAND_DEGREES = 1;

    /** Voltage applied when running toward retract limit (tune sign for your mechanism). */
    public static final double HOMING_SPEED = 0.3;

    // GEAR RATIOS
    public static final double HOOD_SHAFT_REVS_PER_MECH_REV =
        RobotConfigLoader.getDouble("mech.hood_shaft_revs_per_mech_rev");
    public static final double HOOD_GEAR_RATIO =
        RobotConfigLoader.getDouble("mech.hood_gear_ratio");
  }

  public static final class HopperFloorConstants {
    public static final int HOPPER_MOTOR_CAN_ID = 35;
    public static final double HOPPER_FLOOR_SPEED = -0.15; // TODO: tune this value
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_CAN_ID = 9;
    public static final int INTAKE_DEPLOY_MOTOR_CAN_ID = 10;
    public static final int RETRACTED_LIMIT_SWITCH_PORT = 3;
    public static final int DEPLOYED_LIMIT_SWITCH_PORT = 4;

    public static final int DEPLOY_GEARBOX_RATIO = 5;
    public static final double DEPLOY_PULLEY_ONE_GEAR_RATIO = 42.0 / 18.0;
    public static final double DEPLOY_PULLEY_TWO_GEAR_RATIO = 36.0 / 18.0;

    public static final double EXTENDED_ANGLE_DEGREES = 95;
    public static final double RETRACTED_ANGLE_DEGREES = 0;
    public static final double HALF_EXTENDED_ANGLE_DEGREES = 45;

    public static final Rotation2d EXTENDED_POSITION =
        new Rotation2d(Math.toRadians(EXTENDED_ANGLE_DEGREES));
    public static final Rotation2d HALF_EXTENDED_POSITION =
        new Rotation2d(Math.toRadians(HALF_EXTENDED_ANGLE_DEGREES));
    public static final Rotation2d RETRACTED_POSITION =
        new Rotation2d(Math.toRadians(RETRACTED_ANGLE_DEGREES));

    public static final double HOMING_SPEED = -0.4; // TODO tune
    public static final double RETRACTING_SPEED = -0.4; // TODO: tune
    public static final double INTAKING_SPEED = -1; // TODO: tune

    public static final double POSITION_DEADBAND_DEGREES = 3;

    public static final double ROBOT_TO_INTAKE_YAW_DEGREES = 180;
    public static final double DEPLOYED_CURRENT_LIMIT = 10.0; // amps
  }

  public static final class ShooterConstants {
    public static final int LEFT_FLYWHEEL_MOTOR_CAN_ID = 29;
    public static final int RIGHT_FLYWHEEL_MOTOR_CAN_ID = 30;
    public static final int TRANSITION_MOTOR_CAN_ID = 16;
    public static final int LEFT_TRANSITION_MOTOR_CAN_ID = 18;
    public static final int RIGHT_TRANSITION_MOTOR_CAN_ID = 19;

    public static final double TRANSITION_SPEED = 0.4;

    /**
     * Stator current limit for transition rollers (A). Limits torque when a ball is jammed so the
     * motors do not overheat; tune if normal indexing trips the limit.
     */
    public static final double TRANSITION_STATOR_CURRENT_LIMIT_AMPS = 35.0;

    public static final double FLYWHEEL_SPEED_DEADBAND = 2;

    /** Rotor speed (rps) below which StopShooting ends after coasting the flywheel. */
    public static final double FLYWHEEL_COAST_STOPPED_RPS = 4.0;

    public static final double FLYWHEEL_GEAR_RATIO = 24.0 / 18.0;
    public static final double FLYWHEEL_SLIP = 0.255; // TODO: tune?
    public static final double FLYWHEEL_RADIUS_METERS = 0.0508;

    public static final Translation3d BOT_TO_SHOOTER = new Translation3d(0.127, 0, 0.429);

    public static final ShotParameters RED_RIGHT =
        new ShotParameters(
            new Pose2d(14.17, 1.25, new Rotation2d(Math.toRadians(129))),
            new Rotation2d(Math.toRadians(64.4)),
            62.2);
    public static final ShotParameters BLUE_LEFT =
        new ShotParameters(
            Calculations.mirrorPoseAcrossAlliance(RED_RIGHT.pose),
            new Rotation2d(Math.toRadians(64.4)),
            62.2);
    public static final ShotParameters RED_LEFT =
        new ShotParameters(
            Calculations.mirrorPoseAcrossXAxis(RED_RIGHT.pose),
            new Rotation2d(Math.toRadians(64.4)),
            62.2);
    public static final ShotParameters BLUE_RIGHT =
        new ShotParameters(
            Calculations.mirrorPoseAcrossXAxis(BLUE_LEFT.pose),
            new Rotation2d(Math.toRadians(64.4)),
            62.2);
    public static final ShotParameters[] STATIONARY_SHOT_ARRAY = {
      RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT
    };
  }

  /** REV PDH / CTRE PDP CAN ID and logging. */
  public static final class PowerConstants {
    /** CAN ID of the Power Distribution Hub (default REV PDH is 1). */
    public static final int PDH_CAN_ID = 1;
  }

  public static final class TurretConstants {
    public static final int TURRET_MOTOR_CAN_ID = 15;
    public static final int TURRET_BORE_ENCODER_PORT = 1;

    public static final double TURRET_DEADBAND = 0.75;

    public static final double TURRET_ENCODER_OFFSET = 0.54;
    public static final double TURRET_HOMING_ANGLE = 0.0;
    public static final double MIN_TURRET_ANGLE = -285;
    public static final double MAX_TURRET_ANGLE = 105;
  }

  public static final class FieldCoordinates {
    public static final Translation3d BLUE_HUB =
        new Translation3d(
            4.625594, 4.034663, 1.83); // z value is the very top of the hub (to make sure we aren't
    // trying to phase through walls)
    // hub
    public static final Translation3d RED_HUB = new Translation3d(11.915394, 4.034663, 1.80);
    public static final Translation3d BLUE_LEFT_FUNNELING = new Translation3d(2.482, 1.511, 0);
    public static final Translation3d BLUE_RIGHT_FUNNELING = new Translation3d(2.482, 6.653, 0);
    public static final Translation3d RED_LEFT_FUNNELING = new Translation3d(14.858, 6.653, 0);
    public static final Translation3d RED_RIGHT_FUNNELING = new Translation3d(14.858, 1.511, 0);

    public static final Translation2d FIELD_CENTER = new Translation2d(8.270494, 4.034663);
    public static final double BLUE_BUMP_AND_TRENCH_X = 4.626;
    public static final double RED_BUMP_AND_TRENCH_X = 11.915;

    // Tower climb positions (extracted from PathPlanner paths)
    // Blue tower is on the left side of the field (low X), Red tower is on the right (high X)
    public static final Pose2d BLUE_TOWER_LEFT =
        new Pose2d(1.177, 4.697, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_TOWER_RIGHT =
        new Pose2d(1.025, 2.86, Rotation2d.fromDegrees(180));
    public static final Pose2d RED_TOWER_LEFT =
        new Pose2d(15.336, 3.446, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_TOWER_RIGHT =
        new Pose2d(15.488, 5.179, Rotation2d.fromDegrees(180));
  }

  public static final class ShotCalculatorConditions {
    // VALUES YOU WILL WANT TO CHANGE:
    public static final double SHOT_DEADBAND =
        0.30; // smallest calculated error we are okay shooting with
    // shot height measures the highest point of the arc in meters, max should be ceiling height
    // minus a bit, and min should be just over the target height
    public static final double MIN_SHOT_HEIGHT = 2; // 1 for MSLL
    public static final double MV_MAX_SHOT_HEIGHT = 5; // 3.35; // 2 meters for MSLL
    public static final double COMP_MAX_SHOT_HEIGHT = 5;
    public static final double MAX_SHOT_SPEED =
        90
            * ShooterConstants.FLYWHEEL_GEAR_RATIO
            * 2
            * Math.PI
            * ShooterConstants.FLYWHEEL_RADIUS_METERS
            * ShooterConstants.FLYWHEEL_SLIP;
    // in mps, so calculate using flywheel rps * 2 * Math.PI * flywheel radius * flywheel slip

    public static final double VELO_INCREMENT = 0.25; // mps
    public static final double RANGE_INCREMENT = 0.25; // m
    public static final double MAX_COMPONENT_VELO = 1.5 + VELO_INCREMENT; // mps
    public static final double RANGE_FUDGE = 0;
    public static final double MAX_RANGE =
        8; // m //TODO calculate furthest distance we would ever want to shoot from
    // kraken x60 max velocity is ~100 rps

  }
}
