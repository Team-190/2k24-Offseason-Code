package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim;
  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private double AppliedVolts;

  public ArmIOSim() {
    sim =
        new SingleJointedArmSim(
            ArmConstants.ARM_MOTOR_CONFIG,
            ArmConstants.ARM_GEAR_RATIO,
            ArmConstants.ARM_MOMENT_OF_INERTIA,
            ArmConstants.ARM_LENGTH_METERS,
            ArmConstants.ARM_MIN_ANGLE,
            ArmConstants.ARM_MAX_ANGLE,
            true,
            ArmConstants.ARM_MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            ArmConstants.ARM_KP.get(),
            0.0,
            ArmConstants.ARM_KD.get(),
            new Constraints(
                ArmConstants.ARM_MAX_VELOCITY.get(), ArmConstants.ARM_MAX_ACCELERATION.get()));
    feedforward =
        new ArmFeedforward(
            ArmConstants.ARM_KS.get(), ArmConstants.ARM_KG.get(), ArmConstants.ARM_KV.get());

    AppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.armPosition = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.armVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = AppliedVolts;
    inputs.armCurrentAmps = sim.getCurrentDrawAmps();

    inputs.armAbsolutePosition = Rotation2d.fromRadians(sim.getAngleRads());
  }

  @Override
  public void setArmVoltage(double volts) {
    AppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(AppliedVolts);
  }

  @Override
  public void stop() {
    AppliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setArmPosition(double position) {
    AppliedVolts =
        MathUtil.clamp(
            feedback.calculate(sim.getAngleRads(), position)
                + feedforward.calculate(position, feedback.getSetpoint().velocity),
            -12.0,
            12.0);
    sim.setInputVoltage(AppliedVolts);
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    feedback.setPID(kp, ki, kd);
  }

  @Override
  public void setFeedforward(double ks, double kg, double kv) {
    feedforward = new ArmFeedforward(ks, kg, kv);
  }

  @Override
  public void setProfile(double max_velocity, double max_acceleration) {
    feedback.setConstraints(new Constraints(max_velocity, max_acceleration));
  }
}
