// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfiguration;

/** Virtual implementation of a swerve module with two SparkMax controllers and a CANCoder */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax DRIVE_MOTOR;
  private final CANSparkMax AZIMUTH_MOTOR;

  private final RelativeEncoder DRIVE_ENCODER;
  private final RelativeEncoder AZIMUTH_ENCODER;
  private final CANcoder CANCODER;

  private final boolean INVERT_AZIMUTH_MOTOR;

  public ModuleIOSparkMax(ModuleConfiguration configuration) {
    DRIVE_MOTOR = new CANSparkMax(configuration.DRIVE_MOTOR_ID(), MotorType.kBrushless);
    AZIMUTH_MOTOR = new CANSparkMax(configuration.AZIMUTH_MOTOR_ID(), MotorType.kBrushless);

    DRIVE_ENCODER = DRIVE_MOTOR.getEncoder();
    AZIMUTH_ENCODER = AZIMUTH_MOTOR.getEncoder();
    if (DriveConstants.USE_CANIVORE) {
      CANCODER = new CANcoder(configuration.ABSOLUTE_ENCODER_ID(), DriveConstants.CANIVORE_KEY);
    } else {
      CANCODER = new CANcoder(configuration.ABSOLUTE_ENCODER_ID());
    }

    INVERT_AZIMUTH_MOTOR = configuration.INVERT_AZIMUTH_MOTOR();
  }
}
