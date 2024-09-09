// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    inputs = new ClimberIOInputsAutoLogged();
    this.io = io;

  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public boolean limitReached() {
    return inputs.positionMeters >= ClimberConstants.MAX_HEIGHT || inputs.positionMeters <= ClimberConstants.MIN_HEIGHT;
  }

  public Command climb() {
    return Commands.runEnd(()-> io.setVoltage(12), ()->io.stop(), this).until(()->limitReached());
  }

  public Command descend() {
    return Commands.runEnd(()-> io.setVoltage(-12), ()->io.stop(), this).until(()->limitReached());
  }
  
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this);
  }
}
