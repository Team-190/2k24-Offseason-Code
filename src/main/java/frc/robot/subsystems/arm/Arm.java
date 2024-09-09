package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.intake.IntakeIO;

import org.littletonrobotics.junction.Logger;


public class Arm extends SubsystemBase {
  private final ArmIOInputsAutoLogged inputs;
  private final ArmIO io;

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
}
