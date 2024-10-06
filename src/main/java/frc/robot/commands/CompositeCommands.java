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
    return Commands.sequence(
        arm.intakeAngle(), Commands.waitUntil(() -> arm.atSetpoint()), intake.intake());
  }

  public static final Command eject(Intake intake, Arm arm) {
    return Commands.sequence(arm.ejectCommand(), intake.eject());
  }

  public static final Command shootSubwoofer(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setSubwooferVelocity(), arm.subwooferAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setSubwooferVelocity(), arm.subwooferAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootAmp(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setAmpVelocity(), arm.ampAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setAmpVelocity(), arm.ampAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootFeed(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setFeedVelocity(), arm.stowAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setFeedVelocity(), arm.feedAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }
}
