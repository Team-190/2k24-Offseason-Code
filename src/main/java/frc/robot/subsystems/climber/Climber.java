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
  
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    inputs = new ClimberIOInputsAutoLogged();
    this.io = io;
  }

  /** 
   * Updates inputs every 20 ms
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /** 
   * If the climber has reached the maximum height
   * @return if the climber has reached the maximum height
   */
  public boolean upperLimitReached() {
    return inputs.positionMeters >= ClimberConstants.MAX_HEIGHT;
  }

  /** 
   * If the climber has hit the minimum height
  * @return if the climber has reached the minimum height
  */
  public boolean lowerLimitReached() {
    return inputs.positionMeters <= ClimberConstants.MIN_HEIGHT;
  }

  /** 
   * Runs the climber motor at 12 volts until max height is reached. Then stops the motor.
   * @return run climber motor at 12 volts until max height reached
   */
  public Command climb() {
    return Commands.runEnd(() -> io.setVoltage(12), () -> io.stop(), this)
        .until(() -> upperLimitReached());
  }

    /** 
   * Runs the climber motor at -12 volts until minimum height is reached. Then stops the motor.
   * @return run climber motor at -12 volts until minimum height reached
   */
    public Command descend() {
      return Commands.runEnd(() -> io.setVoltage(-12), () -> io.stop(), this)
          .until(() -> lowerLimitReached());
    }
  
    /**
     * Unlatch the climber by running it backwards for 1/4 second
     * @return command to unlock climber
     */
    public Command unlock() {
      return Commands.runEnd(() -> io.setVoltage(-12),()-> io.stop(), this).withTimeout(.25); //TODO: trial and error the time
    }

  /**
   * Stop the motor
   * @return command to stop motor
   */
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this);
  }
}
