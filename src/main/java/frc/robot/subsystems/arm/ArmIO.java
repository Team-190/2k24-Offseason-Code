package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {

    public Rotation2d armPosition = new Rotation2d();
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
    public double armTemperatureCelsius = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}
  
  public default void setArmVoltage(double volts) {}
  
  public default void stop() {}
  
  public default void stowAngle() {}
  
  public default void intakeAngle() {}
  
  public default void ampAngle() {}

  public default void shootAngle() {}
  

}

