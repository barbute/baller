// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for a swerve module */
public interface ModuleIO {
  /** Set of logged inputs from the subsystem */
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveMotorConnected = true;
    public boolean azimuthMotorConnected = true;
    public boolean absoluteEncoderConnected = true;

    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d azimuthAbsolutePosition = new Rotation2d();
    public Rotation2d azimuthPosition = new Rotation2d();
    public double azimuthVelocityRadPerSec = 0.0;
    public double azimuthAppliedVolts = 0.0;
    public double[] azimuthCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the azimuth motor at the specified voltage. */
  public default void setAzimuthVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the azimuth motor. */
  public default void setAzimuthBrakeMode(boolean enable) {}

  /** Set motor to cease motion */
  public default void stop() {}
}
