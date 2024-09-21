// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Initalize robot hardware and input controllers */
public class RobotContainer {
  private final Drive DRIVE;

  private final CommandXboxController PILOT_CONTROLLER = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> AUTONOMOUS_CHOOSER =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        DRIVE =
            new Drive(
                new ModuleIOSparkMax(DriveConstants.MODULE_CONFIGURATIONS[0]),
                new ModuleIOSparkMax(DriveConstants.MODULE_CONFIGURATIONS[1]),
                new ModuleIOSparkMax(DriveConstants.MODULE_CONFIGURATIONS[2]),
                new ModuleIOSparkMax(DriveConstants.MODULE_CONFIGURATIONS[3]),
                new GyroIOPigeon2());
        break;
      case SIM:
        DRIVE =
            new Drive(
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[0]),
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[1]),
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[2]),
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[3]),
                new GyroIO() {});
        break;
      default:
        DRIVE =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        break;
    }

    configureBindings();
    configureAutonomousRoutine();
  }

  /** Set controller bindings here */
  private void configureBindings() {}

  private void configureAutonomousRoutine() {
    AUTONOMOUS_CHOOSER.addDefaultOption("Print Command", new PrintCommand("Hello, world!"));
  }

  /**
   * Return the chosen autonmous command from the selector
   *
   * @return The chosen command
   */
  public Command getAutonomousCommand() {
    return AUTONOMOUS_CHOOSER.get();
  }
}
