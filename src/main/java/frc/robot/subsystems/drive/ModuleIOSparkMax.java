// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfiguration;

/** Virtual implementation of a swerve module with two SparkMax controllers and a CANCoder */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax DRIVE_MOTOR;
  private final CANSparkMax AZIMUTH_MOTOR;

  private final RelativeEncoder DRIVE_ENCODER;
  private final RelativeEncoder AZIMUTH_ENCODER;
  private final CANcoder CANCODER;

  private final boolean INVERT_AZIMUTH_MOTOR;
  private final double DRIVE_MOTOR_GEAR_RATIO;
  private final double AZIMUTH_MOTOR_GEAR_RATIO;

  private final StatusSignal<Double> ABSOLUTE_ENCODER_POSITION;

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
    DRIVE_MOTOR_GEAR_RATIO = configuration.DRIVE_MOTOR_GEAR_RATIO();
    AZIMUTH_MOTOR_GEAR_RATIO = configuration.AZIMUTH_MOTOR_GEAR_RATIO();

    applyConfigurations(configuration);

    ABSOLUTE_ENCODER_POSITION = CANCODER.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, ABSOLUTE_ENCODER_POSITION);
  }

  private void applyConfigurations(ModuleConfiguration configuration) {
    DRIVE_MOTOR.restoreFactoryDefaults();
    AZIMUTH_MOTOR.restoreFactoryDefaults();

    DRIVE_MOTOR.setCANTimeout(DriveConstants.SPARK_CONFIGURATIONS.DRIVE_CAN_TIMEOUT_MS());
    AZIMUTH_MOTOR.setCANTimeout(DriveConstants.SPARK_CONFIGURATIONS.AZIMUTH_CAN_TIMEOUT_MS());

    DRIVE_MOTOR.setInverted(configuration.INVERT_DRIVE_MOTOR());
    AZIMUTH_MOTOR.setInverted(INVERT_AZIMUTH_MOTOR);

    DRIVE_MOTOR.setSmartCurrentLimit(
        DriveConstants.SPARK_CONFIGURATIONS.DRIVE_SMART_CURRENT_LIMIT_AMP());
    AZIMUTH_MOTOR.setSmartCurrentLimit(
        DriveConstants.SPARK_CONFIGURATIONS.AZIMUTH_SMART_CURRENT_LIMIT_AMP());
    DRIVE_MOTOR.enableVoltageCompensation(DriveConstants.SPARK_CONFIGURATIONS.NOMINAL_VOLTAGE());
    AZIMUTH_MOTOR.enableVoltageCompensation(DriveConstants.SPARK_CONFIGURATIONS.NOMINAL_VOLTAGE());

    DRIVE_ENCODER.setPosition(0.0);
    DRIVE_ENCODER.setMeasurementPeriod(
        DriveConstants.SPARK_CONFIGURATIONS.DRIVE_ENCODER_MEASUREMENT_PERIOD_MS());
    DRIVE_ENCODER.setAverageDepth(2);

    AZIMUTH_ENCODER.setPosition(0.0);
    AZIMUTH_ENCODER.setMeasurementPeriod(
        DriveConstants.SPARK_CONFIGURATIONS.AZIMUTH_ENCODER_MEASUREMENT_PERIOD_MS());
    AZIMUTH_ENCODER.setAverageDepth(2);

    DRIVE_MOTOR.setIdleMode(IdleMode.kBrake);
    AZIMUTH_MOTOR.setIdleMode(IdleMode.kCoast);

    DRIVE_MOTOR.setCANTimeout(0);
    AZIMUTH_MOTOR.setCANTimeout(0);

    DRIVE_MOTOR.burnFlash();
    AZIMUTH_MOTOR.burnFlash();

    CANCODER.getConfigurator().apply(new CANcoderConfiguration());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected = DRIVE_MOTOR.getLastError().equals(REVLibError.kOk);
    inputs.azimuthMotorConnected = AZIMUTH_MOTOR.getLastError().equals(REVLibError.kOk);
    inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(ABSOLUTE_ENCODER_POSITION).isOK();

    inputs.drivePositionRad =
        Units.rotationsToRadians(DRIVE_ENCODER.getPosition() / DRIVE_MOTOR_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            DRIVE_ENCODER.getVelocity() / DRIVE_MOTOR_GEAR_RATIO);
    inputs.driveAppliedVolts = DRIVE_MOTOR.getAppliedOutput() * DRIVE_MOTOR.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {DRIVE_MOTOR.getOutputCurrent()};

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(ABSOLUTE_ENCODER_POSITION.getValueAsDouble());
    inputs.azimuthPosition =
        Rotation2d.fromRotations(AZIMUTH_ENCODER.getPosition() / AZIMUTH_MOTOR_GEAR_RATIO);
    inputs.azimuthVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            AZIMUTH_ENCODER.getVelocity() / AZIMUTH_MOTOR_GEAR_RATIO);
    inputs.azimuthAppliedVolts = AZIMUTH_MOTOR.getAppliedOutput() * AZIMUTH_MOTOR.getBusVoltage();
    inputs.azimuthCurrentAmps = new double[] {AZIMUTH_MOTOR.getOutputCurrent()};
  }
}
