package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim topMotorSim;
  private final DCMotorSim bottomMotorSim;
  private final DCMotorSim acceleratorMotorSim;

  private double topAppliedVolts;
  private double bottomAppliedVolts;
  private double acceleratorAppliedVolts;

  public IntakeIOSim() {
    topMotorSim =
        new DCMotorSim(IntakeConstants.TOP_MOTOR_CONFIG, IntakeConstants.TOP_GEAR_RATIO, 0.004);
    bottomMotorSim =
        new DCMotorSim(IntakeConstants.BOTTOM_MOTOR_CONFIG, IntakeConstants.TOP_GEAR_RATIO, 0.004);
    acceleratorMotorSim =
        new DCMotorSim(
            IntakeConstants.ACCELERATOR_MOTOR_CONFIG,
            IntakeConstants.ACCELERATOR_GEAR_RATIO,
            0.004);

    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
    acceleratorAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    topMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    bottomMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    acceleratorMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.topPosition = Rotation2d.fromRadians(topMotorSim.getAngularPositionRad());
    inputs.topVelocityRadPerSec = topMotorSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topCurrentAmps = topMotorSim.getCurrentDrawAmps();

    inputs.bottomPosition = Rotation2d.fromRadians(bottomMotorSim.getAngularPositionRad());
    inputs.bottomVelocityRadPerSec = bottomMotorSim.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomCurrentAmps = bottomMotorSim.getCurrentDrawAmps();

    inputs.acceleratorPosition =
        Rotation2d.fromRadians(acceleratorMotorSim.getAngularPositionRad());
    inputs.acceleratorVelocityRadPerSec = acceleratorMotorSim.getAngularVelocityRadPerSec();
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts;
    inputs.acceleratorCurrentAmps = acceleratorMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setTopVoltage(double volts) {
    topAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    topMotorSim.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    bottomMotorSim.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    acceleratorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    acceleratorMotorSim.setInputVoltage(acceleratorAppliedVolts);
  }

  @Override
  public void stop() {
    topMotorSim.setInputVoltage(0.0);
    bottomMotorSim.setInputVoltage(0.0);
    acceleratorMotorSim.setInputVoltage(0.0);
  }
}
