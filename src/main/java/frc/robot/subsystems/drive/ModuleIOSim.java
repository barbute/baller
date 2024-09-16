// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfiguration;

/** Virtual implementation of a swerve moduler */
public class ModuleIOSim implements ModuleIO {
  private final double LOOP_PERIOD_SEC = 0.02;

  private final DCMotorSim DRIVE_MOTOR;
  private final DCMotorSim AZIMUTH_MOTOR;

  public ModuleIOSim(ModuleConfiguration configuration) {
    DRIVE_MOTOR = new DCMotorSim(DCMotor.getNEO(1), configuration.DRIVE_MOTOR_GEAR_RATIO(), 0.025);
    AZIMUTH_MOTOR =
        new DCMotorSim(DCMotor.getNEO(1), configuration.AZIMUTH_MOTOR_GEAR_RATIO(), 0.004);
  }
}
