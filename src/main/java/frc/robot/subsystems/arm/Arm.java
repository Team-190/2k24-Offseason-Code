package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIOInputsAutoLogged inputs;
  private final ArmIO io;
  private Rotation2d positionSetpoint;
  private boolean isClosedLoop;
  private final SysIdRoutine characterizationRoutine;

  public Arm(ArmIO io) {
    inputs = new ArmIOInputsAutoLogged();
    this.io = io;
    positionSetpoint = ArmConstants.ARM_STOW_CONSTANT;
    isClosedLoop = true;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds.of(1.0)),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Arm/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setArmVoltage(volts.in(Volts)), null, this));
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

    if (isClosedLoop) {
      io.setArmPosition(inputs.armPosition, positionSetpoint);
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

    Logger.recordOutput("Arm/Position", inputs.armPosition.getRadians());
    Logger.recordOutput("Arm/Desired Position", positionSetpoint);
    Logger.recordOutput("Arm/At Setpoint", atSetpoint());
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
          positionSetpoint = ArmConstants.ARM_STOW_CONSTANT;
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
          positionSetpoint = ArmConstants.ARM_INTAKE_CONSTANT;
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
          positionSetpoint = Rotation2d.fromRadians(ArmConstants.ARM_AMP_CONSTANT.get());
        });
  }

  public Command ejectCommand() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          positionSetpoint = ArmConstants.ARM_EJECT_ANGLE;
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
          positionSetpoint =
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
    // return Commands.runOnce(
    //     () -> {
    //       isClosedLoop = true;
    //       positionSetpoint =
    //           Rotation2d.fromRadians(RobotState.getControlData().feedArmAngle().getRadians());
    //     });
    return Commands.none();
  }

  public Command subwooferAngle() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          positionSetpoint =
              shootForward()
                  ? Rotation2d.fromRadians(ArmConstants.ARM_SUBWOOFER_CONSTANT.get())
                  : Rotation2d.fromRadians(
                      ArmConstants.ARM_AMP_CONSTANT.get() + Units.degreesToRadians(3.5));
        });
  }

  public boolean shootForward() {
    double angle = AllianceFlipUtil.apply(RobotState.getRobotPose().getRotation()).getDegrees();
    return (angle > -90 && angle < 90);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public Command runQuasistaticCharacterization(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(direction));
  }

  public Command runDynamicCharacterization(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false), characterizationRoutine.dynamic(direction));
  }

  public Command runVoltage(double volts) {
    return Commands.run(() -> io.setArmVoltage(volts));
  }
}
