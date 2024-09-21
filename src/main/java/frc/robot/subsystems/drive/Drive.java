// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem representing a swerve drive, configurable in DriveConstants.java */
public class Drive extends SubsystemBase {

  /** Organized as FL, FR, BL, BR */
  private final Module[] MODULES = new Module[4];

  private final GyroIO GYRO_IO;
  private final GyroIOInputsAutoLogged GYRO_INPUTS = new GyroIOInputsAutoLogged();

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
  }
}
