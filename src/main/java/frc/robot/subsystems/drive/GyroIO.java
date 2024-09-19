// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for a gyroscope */
public interface GyroIO {
  /** Set of logged inputs from the subsystem */
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
