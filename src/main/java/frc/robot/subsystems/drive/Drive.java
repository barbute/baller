// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.controllers.TeleoperatedController;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
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

  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(
          DriveConstants.DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
          DriveConstants.DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0);
  private static final double MAX_ANGULAR_SPEED_METER_PER_SEC =
      DriveConstants.DRIVE_CONFIGURATION.MAX_LINEAR_VELOCITY_METER_PER_SEC() / DRIVE_BASE_RADIUS;

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
          MAX_ANGULAR_SPEED_METER_PER_SEC);
  private SwerveSetpointGenerator SETPOINT_GENERATOR =
      new SwerveSetpointGenerator(DriveConstants.KINEMATICS, DriveConstants.MODULE_TRANSLATIONS);
  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  private TeleoperatedController teleoperatedController = null;
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
    updatePositions();
    runDriveState();
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

  /** Update pose estimator and odometer. Should ONLY be run in {@link #periodic()} */
  private void updatePositions() {
    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (GYRO_INPUTS.connected) {
      // Use the real gyro angle
      rawGyroRotation = GYRO_INPUTS.yawPosition;
    } else { // Use the angle delta from the kinematics and module deltas
      Twist2d twist = DriveConstants.KINEMATICS.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);
    odometry.update(rawGyroRotation, modulePositions);
  }

  /**
   * Set desired speeds and run desired actions based on the current commanded stated of the drive.
   * Should ONLY be run in {@link #periodic()}
   */
  private void runDriveState() {
    switch (desiredDriveState) {
      case TELEOPERATED:
        if (teleoperatedController != null) {
          desiredSpeeds =
              teleoperatedController.computeChassisSpeeds(
                  poseEstimator.getEstimatedPosition().getRotation(),
                  DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates()),
                  DriveConstants.DRIVE_CONFIGURATION.MAX_LINEAR_VELOCITY_METER_PER_SEC(),
                  MAX_ANGULAR_SPEED_METER_PER_SEC);
        }
        break;
      case TRAJECTORY:
        break;
      case AUTOALIGN:
        break;
      case CHARACTERIZATION:
        desiredSpeeds = null;
        break;
      case SIMPLECHARACTERIZATION:
        desiredSpeeds = null;
        double characterizationVoltage = 3.0;
        runSimpleCharacterization(characterizationVoltage);
        Logger.recordOutput("Drive/SimpleCharacterizationVoltage", characterizationVoltage);
        break;
      case STOPPED:
        desiredSpeeds = null;
        stop();
        break;
      default:
        desiredSpeeds = null;
        break;
    }

    if (desiredSpeeds != null) {
      runSetpoint(desiredSpeeds);
    }
  }

  /**
   * Sets the subsystem's desired state, logic runs in periodic()
   *
   * @param desiredState The desired state
   */
  public void setDriveState(DriveState desiredState) {
    desiredDriveState = desiredState;
    // TODO: Add logic to reset the heading controller when I make that if the state is the heading
    // controller
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runSetpoint(ChassisSpeeds speeds) {
    ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    // TODO Remove this later lol
    boolean areModulesOrienting = false;

    ChassisSpeeds discreteSpeeds = discretize(speeds); // Translational skew compensation
    desiredChassisSpeeds = discreteSpeeds;
    SwerveModuleState[] setpointStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates,
        DriveConstants.DRIVE_CONFIGURATION.MAX_LINEAR_VELOCITY_METER_PER_SEC()); // Normalize speeds

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    if (!areModulesOrienting) {
      currentSetpoint =
          SETPOINT_GENERATOR.generateSetpoint(MODULE_LIMITS, currentSetpoint, discreteSpeeds, 0.02);

      for (int i = 0; i < 4; i++) {
        // Optimized azimuth setpoint angles
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], MODULES[i].getAngle());

        // Prevent jittering from small joystick inputs or noise
        optimizedSetpointStates[i] =
            (Math.abs(
                        optimizedSetpointStates[i].speedMetersPerSecond
                            / DriveConstants.DRIVE_CONFIGURATION
                                .MAX_LINEAR_VELOCITY_METER_PER_SEC())
                    > 0.01)
                ? MODULES[i].runModuleState(optimizedSetpointStates[i])
                : MODULES[i].runModuleState(
                    new SwerveModuleState(
                        optimizedSetpointStates[i].speedMetersPerSecond, MODULES[i].getAngle()));

        // Run state
        MODULES[i].runModuleState(optimizedSetpointStates[i]);
      }
    } else {
      for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] =
            MODULES[i].runModuleState(
                setpointStates[i]); // setDesiredState returns the optimized state
      }
    }

    // Log setpoint states
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    Logger.recordOutput("Drive/SwerveStates/DesiredSpeeds", desiredChassisSpeeds);
  }

  /** Custom method for discretizing swerve speeds */
  private ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  /**
   * Accept the joystick input from the controllers
   *
   * @param xSupplier Forward-backward input
   * @param ySupplier Left-right input
   * @param thetaSupplier Rotational input
   */
  public void acceptTeleroperatedInputs(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
    teleoperatedController = new TeleoperatedController(xSupplier, ySupplier, thetaSupplier);
  }

  /**
   * Runs the drive motors (locked forward) with the applied voltage. Remember that kV for drive is
   * Voltage / Velocity
   *
   * @param volts Voltage to run the drive motors at
   */
  public void runSimpleCharacterization(double volts) {
    for (Module mod : MODULES) {
      mod.runCharacterization(volts);
    }
  }

  /** Stops the drive */
  private void stop() {
    runSetpoint(new ChassisSpeeds());
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = MODULES[i].getState();
    }
    return states;
  }

  /**
   * @return the module positions (turn angles and drive positions) for all of the modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = MODULES[i].getPosition();
    }
    return states;
  }

  /** Returns the current pose estimate with a filter applied */
  @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
  public Pose2d getPoseEstimate() {
    if (xPositionFilter == null || yPositionFilter == null) {
      return poseEstimator.getEstimatedPosition();
    } else {
      return new Pose2d(
          xPositionFilter.calculate(poseEstimator.getEstimatedPosition().getX()),
          yPositionFilter.calculate(poseEstimator.getEstimatedPosition().getY()),
          poseEstimator.getEstimatedPosition().getRotation());
    }
  }

  /** Returns the current pose estimate without a filter applied */
  @AutoLogOutput(key = "Drive/Odometry/UnfilteredPoseEstimate")
  public Pose2d getUnfilteredPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry pose */
  @AutoLogOutput(key = "Drive/Odometry/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }
}
