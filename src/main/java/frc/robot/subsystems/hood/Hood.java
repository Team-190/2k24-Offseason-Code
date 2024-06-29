package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final ProfiledPIDController profiledFeedback =
      new ProfiledPIDController(
          HoodConstants.KP.get(),
          0.0,
          HoodConstants.KD.get(),
          new Constraints(HoodConstants.MAX_VELOCITY.get(), HoodConstants.MAX_ACCELERATION.get()));

  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
    setDefaultCommand(
        run(
            () -> {
              setPosition(HoodConstants.STOWED_POSITION.get());
            }));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (HoodConstants.KP.hasChanged(hashCode())) {
      profiledFeedback.setP(HoodConstants.KP.get());
    }
    if (HoodConstants.KD.hasChanged(hashCode())) {
      profiledFeedback.setD(HoodConstants.KD.get());
    }
    if (HoodConstants.MAX_VELOCITY.hasChanged(hashCode())
        || HoodConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
      profiledFeedback.setConstraints(
          new Constraints(HoodConstants.MAX_VELOCITY.get(), HoodConstants.MAX_ACCELERATION.get()));
    }

    if (DriverStation.isEnabled()) {
      io.setVoltage(profiledFeedback.calculate(inputs.position.getRadians()));
    }

    if (DriverStation.isDisabled()) {
      profiledFeedback.reset(inputs.position.getRadians(), 0);
    }

    Logger.recordOutput("Hood/Goal", profiledFeedback.getGoal().position);
    Logger.recordOutput("Hood/Setpoint", profiledFeedback.getSetpoint().position);
  }

  private void setPosition(double positionRad) {
    double position =
        MathUtil.clamp(
            positionRad + RobotState.getHoodOffset(),
            HoodConstants.MIN_POSITION.get(),
            HoodConstants.MAX_POSITION.get());
    profiledFeedback.setGoal(position);
  }

  public boolean atGoal() {
    return Math.abs(profiledFeedback.getGoal().position - profiledFeedback.getSetpoint().position)
        <= HoodConstants.GOAL_TOLERANCE.get();
  }

  public Command setSourceFeed() {
    return runEnd(
        () -> setPosition(HoodConstants.SOURCE_SIDE_FEED_POSITION.get()),
        () -> setPosition(HoodConstants.STOWED_POSITION.get()));
  }

  public Command setAmpFeed() {
    return runEnd(
        () -> setPosition(HoodConstants.AMP_SIDE_FEED_POSITION.get()),
        () -> setPosition(HoodConstants.STOWED_POSITION.get()));
  }

  public Command setPosePosition(
      Supplier<Translation2d> robotPoseSupplier, Supplier<Translation2d> velocitySupplier) {
    return runEnd(
        () -> {
          setPosition(RobotState.getStateCache().shooterAngle().getRadians());
        },
        () -> setPosition(HoodConstants.STOWED_POSITION.get()));
  }
}
