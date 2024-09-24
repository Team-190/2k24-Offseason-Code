package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(
                  new Pose2d(
                      RobotState.getRobotPose().getTranslation(),
                      AllianceFlipUtil.apply(new Rotation2d())));
            })
        .ignoringDisable(true);
  }

  public static final Command collect(Intake intake, Arm arm) {
    return Commands.parallel(intake.intake(), arm.intakeAngle());
  }

  public static final Command shootSpeaker(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setSpeakerVelocity(), arm.shootAngle()), intake.shoot());
    // .beforeStarting(Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint())));
  }
}
