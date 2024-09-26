package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double velocityRadPerSec = 0;
    public Rotation2d position = new Rotation2d();
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double tempCelsius = 0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}
