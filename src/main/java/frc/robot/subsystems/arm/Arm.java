package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIOInputsAutoLogged inputs;
  private final ArmIO io;
  private Rotation2d desiredAngle;
  private boolean isClosedLoop;

  public Arm(ArmIO io) {
    inputs = new ArmIOInputsAutoLogged();
    this.io = io;
    isClosedLoop = true;
  }

  /**
   * This method is called periodically during the robot's main loop. It updates the arm's input
   * values, processes the inputs for logging, and sets the arm position based on the desired angle
   * if closed-loop control is enabled. Additionally, it updates the PID, feedforward, and profile
   * settings for the arm control.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    desiredAngle = new Rotation2d();

    if (isClosedLoop) {
      io.setArmPosition(desiredAngle.getRadians());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setPID(pid[0], 0.0, pid[1]),
        ArmConstants.ARM_KP,
        ArmConstants.ARM_KD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        feedforward -> io.setFeedforward(feedforward[0], feedforward[1], feedforward[2]),
        ArmConstants.ARM_KS,
        ArmConstants.ARM_KG,
        ArmConstants.ARM_KV);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        profile -> io.setProfile(profile[0], profile[1]),
        ArmConstants.ARM_MAX_ACCELERATION,
        ArmConstants.ARM_MAX_VELOCITY);
  }

  /**
   * Creates a command to set the arm angle to the stow position. The arm angle is set to a constant
   * value defined in the ArmConstants class. The command is executed only once, and the arm enters
   * closed-loop control.
   *
   * @return A command to set the arm angle to the stow position.
   */
  public Command stowAngle() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          desiredAngle = Rotation2d.fromRadians(ArmConstants.ARM_STOW_CONSTANT);
        });
  }

  /**
   * Creates a command to set the arm angle to the intake position. The arm angle is set to a
   * constant value defined in the ArmConstants class. The command is executed only once, and the
   * arm enters closed-loop control.
   *
   * @return A command to set the arm angle to the intake position.
   */
  public Command intakeAngle() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          desiredAngle = Rotation2d.fromRadians(ArmConstants.ARM_INTAKE_CONSTANT);
        });
  }

  /**
   * Creates a command to set the arm angle to the amplifier position. The arm angle is set to a
   * constant value defined in the ArmConstants class. The command is executed only once, and the
   * arm enters closed-loop control.
   *
   * @return A command to set the arm angle to the amplifier position.
   */
  public Command ampAngle() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          desiredAngle = Rotation2d.fromRadians(ArmConstants.ARM_AMP_CONSTANT.get());
        });
  }

  /**
   * Creates a command to set the arm angle to the shoot position. The arm angle is set to the value
   * obtained from the control data's speaker arm angle. The command is executed only once, and the
   * arm enters closed-loop control.
   *
   * @return A command to set the arm angle to the shoot position.
   */
  public Command shootAngle() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          desiredAngle =
              Rotation2d.fromRadians(RobotState.getControlData().speakerArmAngle().getRadians());
        });
  }

  /**
   * Creates a command to set the arm angle to the feed position. The arm angle is set to the value
   * obtained from the control data's feed arm angle. The command is executed only once, and the arm
   * enters closed-loop control.
   *
   * @return A command to set the arm angle to the feed position.
   */
  public Command feedAngle() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          desiredAngle =
              Rotation2d.fromRadians(RobotState.getControlData().feedArmAngle().getRadians());
        });
  }
}
