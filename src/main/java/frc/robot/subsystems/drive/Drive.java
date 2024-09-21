// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem representing a swerve drive, configurable in DriveConstants.java */
public class Drive extends SubsystemBase {

  /** Organized as FL, FR, BL, BR */
  private final Module[] MODULES = new Module[4];

  private final GyroIO GYRO_IO;
  private final GyroIOInputsAutoLogged GYRO_INPUTS = new GyroIOInputsAutoLogged();

  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Use for estimating robot pose with encoders and vision */
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d());
  /**
   * Use to measure robot pose with encoders; This can be used as a fallback if vision is unreliable
   * during comp
   */
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(DriveConstants.KINEMATICS, rawGyroRotation, lastModulePositions);
  /** Used to filter x-pose data from the pose estimator as vision estimates can be a bit shakey */
  private LinearFilter xPositionFilter = LinearFilter.movingAverage(5);
  /** Used to filter y-pose data from the pose estimator as vision estimates can be a bit shakey */
  private LinearFilter yPositionFilter = LinearFilter.movingAverage(5);

  public Drive(
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      GyroIO gyroIO) {
    MODULES[0] = new Module(flModuleIO, 0);
    MODULES[1] = new Module(frModuleIO, 1);
    MODULES[2] = new Module(blModuleIO, 2);
    MODULES[3] = new Module(brModuleIO, 3);
    GYRO_IO = gyroIO;
  }
}
