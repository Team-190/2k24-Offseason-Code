package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
  private final ElevatorSim motorSim;
  private double appliedVolts = 0;

  public ClimberIOSim() {

    motorSim =
        new ElevatorSim(
            ClimberConstants.DC_MOTOR_CONFIG,
            ClimberConstants.GEAR_RATIO,
            ClimberConstants.CARRIAGE_MASS,
            ClimberConstants.DRUM_RADIUS,
            ClimberConstants.MIN_HEIGHT,
            ClimberConstants.MAX_HEIGHT,
            ClimberConstants.GRAVITY,
            ClimberConstants.STARTING_HEIGHT);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.positionMeters = motorSim.getPositionMeters();
    inputs.velocityMPerSec = motorSim.getVelocityMetersPerSecond();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
