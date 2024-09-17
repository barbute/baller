// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** A swerve module */
public class Module {
  private final ModuleIO IO;
  private final ModuleIOInputsAutoLogged INPUTS = new ModuleIOInputsAutoLogged();
  private final int INDEX;

  private final SimpleMotorFeedforward DRIVE_FEEDFORWARD;
  private final PIDController DRIVE_FEEDBACK;
  private final PIDController AZIMUTH_FEEDBACK;
  private Double driveSetpointMeterPerSec = null;
  private Rotation2d azimuthSetpoint = null;

  private Rotation2d azimuthRelativeOffset = null; // Relative + Offset = Absolute

  private LoggedTunableNumber driveFeedbackP;
  private LoggedTunableNumber driveFeedbackI;
  private LoggedTunableNumber driveFeedbackD;

  private LoggedTunableNumber azimuthFeedbackP;
  private LoggedTunableNumber azimuthFeedbackI;
  private LoggedTunableNumber azimuthFeedbackD;

  public Module(ModuleIO io, int index) {
    this.IO = io;
    this.INDEX = index;

    DRIVE_FEEDFORWARD =
        new SimpleMotorFeedforward(
            DriveConstants.MODULE_GAINS.DRIVE_S(),
            DriveConstants.MODULE_GAINS.DRIVE_V(),
            DriveConstants.MODULE_GAINS.DRIVE_A());
    DRIVE_FEEDBACK =
        new PIDController(
            DriveConstants.MODULE_GAINS.DRIVE_P(),
            DriveConstants.MODULE_GAINS.DRIVE_I(),
            DriveConstants.MODULE_GAINS.DRIVE_D());
    AZIMUTH_FEEDBACK =
        new PIDController(
            DriveConstants.MODULE_GAINS.AZIMUTH_P(),
            DriveConstants.MODULE_GAINS.AZIMUTH_I(),
            DriveConstants.MODULE_GAINS.AZIMUTH_D());

    driveFeedbackP = new LoggedTunableNumber("Drive/Tuning/DriveP", DRIVE_FEEDBACK.getP());
    driveFeedbackI = new LoggedTunableNumber("Drive/Tuning/DriveI", DRIVE_FEEDBACK.getI());
    driveFeedbackD = new LoggedTunableNumber("Drive/Tuning/DriveD", DRIVE_FEEDBACK.getD());

    azimuthFeedbackP = new LoggedTunableNumber("Drive/Tuning/AzimuthP", AZIMUTH_FEEDBACK.getP());
    azimuthFeedbackI = new LoggedTunableNumber("Drive/Tuning/AzimuthI", AZIMUTH_FEEDBACK.getI());
    azimuthFeedbackD = new LoggedTunableNumber("Drive/Tuning/AzimuthD", AZIMUTH_FEEDBACK.getD());

    AZIMUTH_FEEDBACK.enableContinuousInput(-Math.PI, Math.PI);
    // TODO Set brake mode here
  }

  /** Called in the drive subsystem's periodic loop */
  public void periodic() {
    IO.updateInputs(INPUTS);
    Logger.processInputs("Drive/Module" + Integer.toString(INDEX), INPUTS);

    // On first cycle, reset relative azimuth encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (azimuthRelativeOffset == null && INPUTS.azimuthAbsolutePosition.getRadians() != 0.0) {
      azimuthRelativeOffset = INPUTS.azimuthAbsolutePosition.minus(INPUTS.azimuthPosition);
    }

    // Run closed loop azimuth control
    if (azimuthSetpoint != null) {
      double azimuthFeedbackOutput =
          AZIMUTH_FEEDBACK.calculate(getAngle().getRadians(), azimuthSetpoint.getRadians());
      IO.setAzimuthVoltage(azimuthFeedbackOutput);

      // Run closed loop drive control
      // Only allowed if closed loop azimuth control is running
      if (driveSetpointMeterPerSec != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustedDriveSetpoint =
            driveSetpointMeterPerSec * Math.cos(AZIMUTH_FEEDBACK.getPositionError());
        double velocitySetpointRadPerSec =
            adjustedDriveSetpoint / DriveConstants.DRIVE_CONFIGURATION.WHEEL_RADIUS_METER();

        double driveFeedforwardOutput = DRIVE_FEEDFORWARD.calculate(velocitySetpointRadPerSec);
        double driveFeedbackOutput =
            DRIVE_FEEDBACK.calculate(INPUTS.driveVelocityRadPerSec, velocitySetpointRadPerSec);

        IO.setDriveVoltage(driveFeedforwardOutput + driveFeedbackOutput);

        if (Constants.DEBUGGING_MODE_ENABLED) {
          Logger.recordOutput("Drive/Module/VelocitySetpointRadPerSec", velocitySetpointRadPerSec);
          Logger.recordOutput("Drive/Module/FeedbackOutput", driveFeedbackOutput);
          Logger.recordOutput("Drive/Module/FeedforwardOutput", driveFeedforwardOutput);
          Logger.recordOutput("Drive/Module/FeedbackError", DRIVE_FEEDBACK.getPositionError());
          Logger.recordOutput("Drive/Module/FeedbackSetpoint", DRIVE_FEEDBACK.getSetpoint());
        }
      }
    }

    // Update feedback controller gains
    if (Constants.DEBUGGING_MODE_ENABLED) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () ->
              setDriveFeedbackGains(
                  driveFeedbackP.get(), driveFeedbackI.get(), driveFeedbackD.get()),
          driveFeedbackP,
          driveFeedbackI,
          driveFeedbackD);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () ->
              setAzimuthFeedbackGains(
                  azimuthFeedbackP.get(), azimuthFeedbackI.get(), azimuthFeedbackD.get()),
          azimuthFeedbackP,
          azimuthFeedbackI,
          azimuthFeedbackD);
    }
  }

  /**
   * Runs the module with the specified setpoint state
   * 
   * @param desiredState The specified setpoint state of the module
   * @return The optimized state
   */
  public SwerveModuleState runModuleState(SwerveModuleState desiredState) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(desiredState, getAngle());

    // Update setpoints, controllers run in "periodic"
    azimuthSetpoint = optimizedState.angle;
    driveSetpointMeterPerSec = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Set the drive motor's feedback gains */
  private void setDriveFeedbackGains(double P, double I, double D) {
    DRIVE_FEEDBACK.setPID(P, I, D);
  }

  /** Set the azimuth motor's feedback gains */
  private void setAzimuthFeedbackGains(double P, double I, double D) {
    AZIMUTH_FEEDBACK.setPID(P, I, D);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (azimuthRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return INPUTS.azimuthPosition.plus(azimuthRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return INPUTS.drivePositionRad * DriveConstants.DRIVE_CONFIGURATION.WHEEL_RADIUS_METER();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return INPUTS.driveVelocityRadPerSec * DriveConstants.DRIVE_CONFIGURATION.WHEEL_RADIUS_METER();
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return INPUTS.driveVelocityRadPerSec;
  }
}
