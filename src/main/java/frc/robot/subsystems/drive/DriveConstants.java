// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.characterization.DriveIdentificationRoutine.MotorType;

/** Swerve constants class */
public class DriveConstants {
  public record DriveConfiguration(
      int GYRO_ID,
      double WHEEL_RADIUS_METER,
      double TRACK_WIDTH_X_METER,
      double TRACK_WIDTH_Y_METER,
      double MAX_LINEAR_VELOCITY_METER_PER_SEC,
      MotorType MOTOR_TYPE) {}

  public record ModuleConfiguration(
      int DRIVE_MOTOR_ID,
      int AZIMUTH_MOTOR_ID,
      int ABSOLUTE_ENCODER_ID,
      Rotation2d ABSOLUTE_ENCODER_OFFSET,
      boolean INVERT_DRIVE_MOTOR,
      boolean INVERT_AZIMUTH_MOTOR,
      double DRIVE_MOTOR_GEAR_RATIO,
      double AZIMUTH_MOTOR_GEAR_RATIO) {}

  public record SparkMaxModuleConfiguration(
      int DRIVE_CAN_TIMEOUT_MS,
      int AZIMUTH_CAN_TIMEOUT_MS,
      int DRIVE_SMART_CURRENT_LIMIT_AMP,
      int AZIMUTH_SMART_CURRENT_LIMIT_AMP,
      int DRIVE_SECONDARY_CURRENT_LIMIT_AMP,
      int AZIMUTH_SECONDARY_CURRENT_LIMIT_AMP,
      double NOMINAL_VOLTAGE,
      int DRIVE_ENCODER_MEASUREMENT_PERIOD_MS,
      int AZIMUTH_ENCODER_MEASUREMENT_PERIOD_MS,
      int CAN_TIMEOUT_MS) {}

  public record ModuleGains(
      double DRIVE_S,
      double DRIVE_V,
      double DRIVE_A,
      double DRIVE_P,
      double DRIVE_I,
      double DRIVE_D,
      double AZIMUTH_P,
      double AZIMUTH_I,
      double AZIMUTH_D) {}

  public static final String CANIVORE_KEY = "CTREBUS";
  public static final boolean USE_CANIVORE = true;

  public static final DriveConfiguration DRIVE_CONFIGURATION =
      switch (Constants.CURRENT_MODE) {
        case REAL, SIM -> new DriveConfiguration(
            10, // Gyro ID
            5.08 / 100.0, // Wheel radius TODO Check this bruh
            Units.inchesToMeters(24.25), // Track width X
            Units.inchesToMeters(24.25), // Track width Y
            Units.feetToMeters(14.5),
            MotorType.NEO); // Max linear velocity M/S
        default -> new DriveConfiguration(10, 0.0, 0.0, 0.0, 0.0, MotorType.NEO);
      };

  public static final SparkMaxModuleConfiguration SPARK_CONFIGURATIONS =
      new SparkMaxModuleConfiguration(250, 250, 60, 40, 80, 100, 12.0, 20, 10, 0);

  public static final ModuleConfiguration[] MODULE_CONFIGURATIONS =
      switch (Constants.CURRENT_MODE) {
          // Drive motor ID, Azimuth motor ID, Absolute encoder ID, Absolute encoder offset, Invert
          // azimuth, Drive gearing, Azimuth gearing
        case REAL -> new ModuleConfiguration[] {
          new ModuleConfiguration(
              11, 21, 31, Rotation2d.fromRadians(-1.738), false, true, 6.75 / 1.0, 150.0 / 7.0),
          new ModuleConfiguration(
              12, 22, 32, Rotation2d.fromRadians(-1.705), false, true, 6.75 / 1.0, 150.0 / 7.0),
          new ModuleConfiguration(
              13, 23, 33, Rotation2d.fromRadians(-2.538), false, true, 6.75 / 1.0, 150.0 / 7.0),
          new ModuleConfiguration(
              14, 24, 34, Rotation2d.fromRadians(2.431), false, true, 6.75 / 1.0, 150.0 / 7.0)
        };
        default -> {
          ModuleConfiguration[] configurations = new ModuleConfiguration[4];
          for (int i = 0; i < 4; i++) {
            configurations[i] =
                new ModuleConfiguration(0, 0, 0, new Rotation2d(), false, false, 6.746, 21.429);
          }
          yield configurations; // "Return" configurations statement but for arrow function, more
          // complex explanation but am too lazy rn
        }
      };

  public static final ModuleGains MODULE_GAINS =
      // Drive: S, V, A, P, I, D | Azimuth: P, I, D
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ModuleGains(0.14287, 0.13772, 0.012954, 0.1, 0.0, 0.0, 8.0, 0.0, 0.0);
        case SIM -> new ModuleGains(0.0, 2.06 / 15.32, 0.0, 0.1, 0.0, 0.0, 5.0, 0.0, 0.0);
        default -> new ModuleGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(
            DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
        new Translation2d(
            DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            -DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
        new Translation2d(
            -DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
        new Translation2d(
            -DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            -DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
      };

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);
}
