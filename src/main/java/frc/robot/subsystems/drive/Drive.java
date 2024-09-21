// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem representing a swerve drive, configurable in DriveConstants.java */
public class Drive extends SubsystemBase {
  /** All possible states the drive subsystem can be in */
  public enum DriveState {
    /** Driving with input from driver controllers */
    TELEOPERATED,
    /** Driving based on a preplanned trajectory */
    TRAJECTORY,
    /** Driving to a location on a field automatically */
    AUTOALIGN,
    /** Characterizing */
    CHARACTERIZATION,
    /** Only runs drive volts, kV = voltage / velocity; sets the drive volts to 1.0 */
    SIMPLECHARACTERIZATION,
    /** Drivetrain is commanded to do nothing */
    STOPPED
  }

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

  // TODO Check if limits are correct with drivetrain model
  private final ModuleLimits MODULE_LIMITS =
      new ModuleLimits(
          DriveConstants.DRIVE_CONFIGURATION.MAX_LINEAR_VELOCITY_METER_PER_SEC(),
          DriveConstants.DRIVE_CONFIGURATION.MAX_LINEAR_VELOCITY_METER_PER_SEC() * 5.0,
          DriveConstants.DRIVE_CONFIGURATION.MAX_ANGULAR_VELOCITY_RAD_PER_SEC());
  private SwerveSetpointGenerator SETPOINT_GENERATOR =
      new SwerveSetpointGenerator(DriveConstants.KINEMATICS, DriveConstants.MODULE_TRANSLATIONS);
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private DriveState desiredDriveState = DriveState.STOPPED;

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

    // Configure setpoint generator
    // TODO Why am I instantiating this twice
    SETPOINT_GENERATOR =
        SwerveSetpointGenerator.builder()
            .kinematics(DriveConstants.KINEMATICS)
            .moduleLocations(DriveConstants.MODULE_TRANSLATIONS)
            .build();

    // TODO Configure PathPlanner later ig

    // TODO Add SysID configuration here later
  }

  @Override
  public void periodic() {
    GYRO_IO.updateInputs(GYRO_INPUTS);
    Logger.processInputs("Drive/Gyro", GYRO_INPUTS);
    for (var module : MODULES) {
      module.periodic();
    }

    runDisabledChecks();
  }

  /** Handle edge cases when driver stations is disabled. Runs in {@link #periodic()} */
  private void runDisabledChecks() {
    if (DriverStation.isDisabled()) {
      for (var module : MODULES) {
        module.stop();
      }

      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
  }
}
