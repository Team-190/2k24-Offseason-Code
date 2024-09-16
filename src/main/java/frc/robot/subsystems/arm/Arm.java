package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

import org.littletonrobotics.junction.Logger;


public class Arm extends SubsystemBase {
  private final ArmIOInputsAutoLogged inputs;
  private final ArmIO io;
  private Rotation2d desiredAngle;


  public Arm(ArmIO io) {
    inputs = new ArmIOInputsAutoLogged();
    this.io = io;
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }
  
  public Command stowAngle() {
    return Commands.run(() -> io.stowAngle());
  }
  
  public Command intakeAngle() {
    return Commands.run(() -> io.intakeAngle());
  }
  
  public Command ampAngle() {
    return Commands.run(() -> io.ampAngle());
  }

  public Command shootAngle() {
    return Commands.run(() -> io.shootAngle());
  }

  public boolean atDesiredAngle() {
    //TODO: Check if the value outputted is correct and change getRotations() to getRadians() or getDegrees() if needed
    return Math.abs(desiredAngle.getRotations() - io.getCurrentAngle())
        <= ArmConstants.GOAL_TOLERANCE;
  }

  public void setDesiredAngle(double newAngle) {
    desiredAngle = new Rotation2d(newAngle);
  }
  
  public Rotation2d getDesiredAngle() {
    return desiredAngle;
  }
}
