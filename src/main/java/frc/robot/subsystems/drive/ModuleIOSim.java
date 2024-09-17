// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfiguration;

/** Virtual implementation of a swerve moduler */
public class ModuleIOSim implements ModuleIO {
  /** How long it takes the simulator to update */
  private final double LOOP_PERIOD_SEC = 0.02;

  private final DCMotorSim DRIVE_MOTOR;
  private final DCMotorSim AZIMUTH_MOTOR;

  /** "faking" the initial position of an absolute encoder to simulate that aspect of the azimuth position */
  private final Rotation2d ABSOLUTE_INITIAL_POSITION = new Rotation2d(Math.random() * 2.0 * Math.PI);

  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  public ModuleIOSim(ModuleConfiguration configuration) {
    DRIVE_MOTOR = new DCMotorSim(DCMotor.getNEO(1), configuration.DRIVE_MOTOR_GEAR_RATIO(), 0.025);
    AZIMUTH_MOTOR =
        new DCMotorSim(DCMotor.getNEO(1), configuration.AZIMUTH_MOTOR_GEAR_RATIO(), 0.004);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    
  }
}
