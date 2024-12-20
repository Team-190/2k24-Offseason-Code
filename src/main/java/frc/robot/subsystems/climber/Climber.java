// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private boolean isUnlocked;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    inputs = new ClimberIOInputsAutoLogged();
    this.io = io;
  }

  /** Updates inputs every 20 ms */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public boolean isClimbed() {
    return inputs.isClimbed && isUnlocked;
  }

  /**
   * Runs the climber motor at 12 volts until max height is reached. Then stops the motor.
   *
   * @return run climber motor at 12 volts until max height reached
   */
  public Command climb() {
    return Commands.runEnd(
            () -> io.setVoltage(12.0), () -> io.setVoltage(ClimberConstants.HOLD_VOLTAGE), this)
        .until(() -> inputs.isClimbed);
  }

  public Command deClimb() {
    return Commands.runEnd(() -> io.setVoltage(-12.0), () -> io.setVoltage(0.0), this)
        .until(() -> inputs.position.getRadians() <= 0.0);
  }

  /**
   * Creates a command to descend the climber. The climber motor is run in reverse at 12 volts until
   * the specified release position is reached. Once the release position is reached, the motor is
   * stopped.
   *
   * @return a command to descend the climber
   */
  public Command descend() {
    return Commands.runEnd(() -> io.setVoltage(-12.0), () -> io.stop(), this)
        .until(
            () -> inputs.position.getRadians() <= ClimberConstants.RELEASE_POSITION.getRadians());
  }

  /**
   * Unlatch the climber by running it backwards for 1 rotation of the spool
   *
   * @return command to unlock climber
   */
  public Command unlock() {
    return Commands.runEnd(() -> io.setVoltage(12.0), () -> io.stop(), this)
        .until(() -> inputs.position.getRadians() >= ClimberConstants.RELEASE_POSITION.getRadians())
        .finallyDo(() -> isUnlocked = true);
  }

  /**
   * Stop the motor
   *
   * @return command to stop motor
   */
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this);
  }
}
