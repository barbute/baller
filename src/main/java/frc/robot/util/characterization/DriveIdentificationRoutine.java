// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.characterization;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.util.debugging.Alert;
import frc.robot.util.debugging.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * System Identification (SysID) can be used to characterize a mechanism, and generate control
 * formula constants from test data
 */
public class DriveIdentificationRoutine {
  // For rev logs extract using wpilib's data log tool:
  // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
  // For talon logs extract using phoenix tuner x:
  // https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html

  /** Voltages and loggers vary with different motors */
  public enum MotorType {
    NEO,
    FALCON500,
    KRAKENX60
  }

  /**
   * Return a command that runs the System Identification tests
   *
   * @param drive The drive subsystem instance
   * @param modules The swerve modules to apply voltage to
   * @param motorType
   * @return A squential command that runs the tests
   */
  public Command executeIdentificationRoutineCommand(
      Drive drive, Module[] modules, MotorType motorType) {
    // Initialize defaults
    var rampRateVoltsPerSec = Units.Volts.of(0).per(Units.Seconds.of(0));
    var stepVoltage = Units.Volts.of(0);
    var timeoutSeconds = Units.Seconds.of(1);

    // Pretty much same for now :/
    switch (motorType) {
      case NEO:
        rampRateVoltsPerSec = Units.Volts.of(1).per(Units.Seconds.of(1));
        stepVoltage = Units.Volts.of(3);
        timeoutSeconds = Units.Seconds.of(5);
        break;
      case FALCON500:
        rampRateVoltsPerSec = Units.Volts.of(1).per(Units.Seconds.of(1));
        stepVoltage = Units.Volts.of(3);
        timeoutSeconds = Units.Seconds.of(5);
        break;
      case KRAKENX60:
        rampRateVoltsPerSec = Units.Volts.of(1).per(Units.Seconds.of(1));
        stepVoltage = Units.Volts.of(3);
        timeoutSeconds = Units.Seconds.of(5);
        break;
      default:
        // Assume we are using NEOs by default
        rampRateVoltsPerSec = Units.Volts.of(1).per(Units.Seconds.of(1));
        stepVoltage = Units.Volts.of(3);
        timeoutSeconds = Units.Seconds.of(5);
        break;
    }

    // Configure a new routine
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRateVoltsPerSec,
                stepVoltage,
                timeoutSeconds,
                (state) -> recordState("Drive/SysIDState", state.toString(), motorType)),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    // We call run characterization for the modules as it uses PID control to lock
                    // the azimuths forward
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null, // Logged using external loggers (URCL or Phoenix)
                drive));

    // Setup tests
    SequentialCommandGroup driveCharacterizationCommand =
        new SequentialCommandGroup(
            Commands.runOnce(() -> startPhoenixLogger(motorType)),
            Commands.waitSeconds(3.0),
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(3.0),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(3.0),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(3.0),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(
                () ->
                    new Alert("ROBOT", "Drive characterization complete", AlertType.INFO)
                        .set(true)),
            Commands.runOnce(() -> stopPhoenixLogger(motorType)));

    return driveCharacterizationCommand;
  }

  /**
   * Starts the CTRE signal logger
   *
   * @param motorType Type of motor to check if we should start the logger or not
   * @return Command that starts the logger
   */
  private Command startPhoenixLogger(MotorType motorType) {
    return Commands.runOnce(
        () -> {
          switch (motorType) {
            case NEO:
              break;
            default:
              SignalLogger.start();
          }
        });
  }

  /**
   * Stops the CTRE signal logger
   *
   * @param motorType Type of motor to check if the logger has been started or not
   * @return Command that stops the logger
   */
  private Command stopPhoenixLogger(MotorType motorType) {
    return Commands.runOnce(
        () -> {
          switch (motorType) {
            case NEO:
              break;
            default:
              SignalLogger.stop();
          }
        });
  }

  /**
   * Method to record the state of the System Identification routine, handles different motors
   * separately
   *
   * @param key Logging path
   * @param state The actual state of the routine
   * @param motorType The motors the test is being run on
   */
  private void recordState(String key, String state, MotorType motorType) {
    switch (motorType) {
      case NEO:
        Logger.recordOutput(key, state);
        break;
      case FALCON500:
        if (Constants.CURRENT_MODE == Mode.SIM) {
          Logger.recordOutput(key, state);
        } else {
          SignalLogger.writeString(key, state);
        }
        break;
      case KRAKENX60:
        if (Constants.CURRENT_MODE == Mode.SIM) {
          Logger.recordOutput(key, state);
        } else {
          SignalLogger.writeString(key, state);
        }
        break;
      default:
        break;
    }
  }
}
