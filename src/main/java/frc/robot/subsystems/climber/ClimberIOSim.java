package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
  private final ElevatorSim motorSim;
  private double appliedVolts;

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
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECONDS);
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
  public void setPosition(double position) {
    if (motorSim.getPositionMeters() <= position) {
      appliedVolts = 12.0;
      motorSim.setInputVoltage(appliedVolts);
    } else {
      appliedVolts = 0.0;
      motorSim.setInputVoltage(appliedVolts);
    }
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}