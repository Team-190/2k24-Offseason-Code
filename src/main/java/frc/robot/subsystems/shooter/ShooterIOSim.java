package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LinearProfile;

public class ShooterIOSim implements ShooterIO {

  private DCMotorSim topMotorSim;
  private DCMotorSim bottomMotorSim;
  private SimpleMotorFeedforward feedForward;
  private final PIDController feedback;
  private final LinearProfile profile;
  private double topAppliedVolts;
  private double bottomAppliedVolts;

  public ShooterIOSim() {

    topMotorSim =
        new DCMotorSim(
            ShooterConstants.TOP_MOTOR_CONFIG,
            ShooterConstants.TOP_GEAR_RATIO,
            ShooterConstants.TOP_MOMENT_OF_INERTIA);
    bottomMotorSim =
        new DCMotorSim(
            ShooterConstants.BOTTOM_MOTOR_CONFIG,
            ShooterConstants.BOTTOM_GEAR_RATIO,
            ShooterConstants.BOTTOM_MOMENT_OF_INERTIA);
    feedForward = new SimpleMotorFeedforward(ShooterConstants.KS.get(), ShooterConstants.KV.get());
    feedback = new PIDController(ShooterConstants.KP.get(), 0.0, ShooterConstants.KD.get());
    profile =
        new LinearProfile(
            ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get(),
            Constants.LOOP_PERIOD_SECONDS);
    feedback.setTolerance(ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND);
    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    topMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    bottomMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    inputs.topPosition = Rotation2d.fromRadians(topMotorSim.getAngularPositionRad());
    inputs.bottomPosition = Rotation2d.fromRadians(bottomMotorSim.getAngularPositionRad());
    inputs.topVelocityRadPerSec = topMotorSim.getAngularVelocityRadPerSec();
    inputs.bottomVelocityRadPerSec = bottomMotorSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.bottomAppliedVolts = bottomAppliedVolts;

    inputs.topVelocityGoalRadiansPerSec = profile.getGoal();
    inputs.bottomVelocityGoalRadiansPerSec = profile.getGoal();

    inputs.topVelocitySetpointRadiansPerSec = feedback.getSetpoint();
    inputs.bottomVelocitySetpointRadiansPerSec = feedback.getSetpoint();
  }

  @Override
  public void setTopVelocitySetPoint(double setPointVelocityRadiansPerSecond) {
    profile.setGoal(setPointVelocityRadiansPerSecond, topMotorSim.getAngularVelocityRadPerSec());
    topAppliedVolts =
        MathUtil.clamp(
            feedback.calculate(
                    topMotorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
                + feedForward.calculate(setPointVelocityRadiansPerSecond),
            -12.0,
            12.0);
    topMotorSim.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVelocitySetPoint(double setPointVelocityRadiansPerSecond) {
    profile.setGoal(setPointVelocityRadiansPerSecond, bottomMotorSim.getAngularVelocityRadPerSec());
    bottomAppliedVolts =
        MathUtil.clamp(
            feedback.calculate(
                    bottomMotorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
                + feedForward.calculate(setPointVelocityRadiansPerSecond),
            -12.0,
            12.0);
    bottomMotorSim.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void setVoltage(double volts) {

    topAppliedVolts = MathUtil.clamp(volts, -12, 12);
    bottomAppliedVolts = MathUtil.clamp(volts, -12, 12);
    topMotorSim.setInputVoltage(topAppliedVolts);
    bottomMotorSim.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setTopPID(double kP, double kI, double kD) {

    feedback.setPID(kP, kI, kD);
  }

  @Override
  public void setBottomPID(double kP, double kI, double kD) {

    feedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTopFeedForward(double kS, double kV, double kA) {

    feedForward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setBottomFeedForward(double kS, double kV, double kA) {

    feedForward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setTopProfile(double maxAcceleration) {
    profile.setMaxAcceleration(maxAcceleration);
  }

  @Override
  public void setBottomProfile(double maxAcceleration) {
    profile.setMaxAcceleration(maxAcceleration);
  }

  @Override
  public boolean atSetPoint() {
    return (Math.abs(profile.getGoal() - feedback.getSetpoint())
        <= ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND);
  }

  @Override
  public void stop() {

    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
    topMotorSim.setInputVoltage(0.0);
    bottomMotorSim.setInputVoltage(0.0);
  }
}
