// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private Double driveSetpointRadPerSec = null;
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
  }
}
