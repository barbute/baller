// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class RobotContainer {
  private final LoggedDashboardChooser<Command> AUTONOMOUS_CHOOSER =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
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
