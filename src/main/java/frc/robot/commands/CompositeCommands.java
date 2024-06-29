package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TrackingMode;

public class CompositeCommands {
  public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
      new PathConstraints(5.0, 4.0, 540.0, 720.0);

  public static final Command resetHeading() {
    return Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    new Pose2d(RobotState.getRobotPose().getTranslation(), new Rotation2d())))
        .ignoringDisable(true);
  }

  public static final Command getCollectCommand(
      Intake intake, Serializer serializer, Vision noteVision, Vision aprilTagVision) {
    return Commands.sequence(
        intake.deployIntake(),
        Commands.race(intake.runVoltage(), serializer.intake()),
        intake.retractIntake());
  }

  public static final Command getCollectCommand(Intake intake, Serializer serializer) {
    return Commands.sequence(
        intake.deployIntake(),
        Commands.race(intake.runVoltage(), serializer.intake()),
        intake.retractIntake());
  }

  public static final Command getOuttakeCommand(
      Intake intake, Serializer serializer, Kicker kicker) {
    return Commands.parallel(intake.outtake(), serializer.outtake(), kicker.outtake());
  }

  public static final Command getSourceFeedCommand(
      Shooter shooter, Hood hood, Accelerator accelerator, Kicker kicker) {
    return Commands.parallel(
        shooter.runSourceFeed(), hood.setSourceFeed(), accelerator.runAccelerator());
  }

  public static final Command getAmpFeedCommand(
      Shooter shooter, Hood hood, Accelerator accelerator, Kicker kicker) {
    return Commands.parallel(shooter.runAmpFeed(), hood.setAmpFeed(), accelerator.runAccelerator());
  }

  public static final Command getPosePrepShooterCommand(
      Drive drive, Hood hood, Shooter shooter, Accelerator accelerator) {
    return Commands.parallel(
        shooter.runPoseDistance(
            () -> RobotState.getRobotPose().getTranslation(), drive::getFieldRelativeVelocity),
        hood.setPosePosition(
            () -> RobotState.getRobotPose().getTranslation(), drive::getFieldRelativeVelocity),
        accelerator.runAccelerator());
  }

  public static final Command getShootCommand(Intake intake, Serializer serializer, Kicker kicker) {
    return Commands.parallel(serializer.shoot(), intake.runVoltage(), kicker.shoot())
        .withTimeout(0.25);
  }

  public static final Command getFeedCommand(Intake intake, Serializer serializer, Kicker kicker) {
    return Commands.sequence(
        intake.deployIntake(),
        Commands.parallel(intake.runVoltage(), serializer.intake(), kicker.shoot()));
  }

  public static Command increaseFlywheelVelocity() {
    return Commands.runOnce(
        () -> RobotState.setFlywheelOffset(RobotState.getFlywheelOffset() + 10));
  }

  public static Command decreaseFlywheelVelocity() {
    return Commands.runOnce(
        () -> RobotState.setFlywheelOffset(RobotState.getFlywheelOffset() - 10));
  }

  public static Command increaseHoodAngle() {
    return Commands.runOnce(
        () -> RobotState.setHoodOffset(RobotState.getHoodOffset() + Units.degreesToRadians(0.25)));
  }

  public static Command decreaseHoodAngle() {
    return Commands.runOnce(
        () -> RobotState.setHoodOffset(RobotState.getHoodOffset() - Units.degreesToRadians(0.25)));
  }

  public static final Command getTrackNoteSpikeCommand(
      Drive drive,
      Intake intake,
      Serializer serializer,
      Pose2d targetPose,
      TrackingMode targetType) {
    return Constants.ROBOT.equals(RobotType.ROBOT_SIM)
        ? Commands.parallel(
                DriveCommands.moveTowardsTarget(
                    drive,
                    FieldConstants.StagingLocations.spikeX - 0.5,
                    AllianceFlipUtil.apply(targetPose),
                    targetType),
                getCollectCommand(intake, serializer))
            .withTimeout(2)
        : Commands.race(
                DriveCommands.moveTowardsTarget(
                    drive,
                    FieldConstants.StagingLocations.spikeX - 0.5,
                    AllianceFlipUtil.apply(targetPose),
                    targetType),
                getCollectCommand(intake, serializer))
            .withTimeout(2);
  }

  public static final Command getAimSpeakerCommand(Drive drive) {
    return DriveCommands.aimTowardsTarget(drive);
  }

  public static final Command getPath(Pose2d endingPose) {
    return AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(endingPose), DEFAULT_PATH_CONSTRAINTS);
  }

  public static final Command getPath(Pose2d endingPose, PathConstraints pathConstraints) {
    return AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(endingPose), pathConstraints);
  }

  public static final Command getPath(Drive drive, Pose2d startingPose, Pose2d endingPose) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(AllianceFlipUtil.apply(startingPose));
              drive.setPose(AllianceFlipUtil.apply(startingPose));
            }),
        AutoBuilder.pathfindToPose(
            DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? endingPose
                : AllianceFlipUtil.apply(endingPose),
            DEFAULT_PATH_CONSTRAINTS));
  }

  public static final Command getPath(
      Drive drive, Pose2d startingPose, Pose2d endingPose, PathConstraints pathConstraints) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(AllianceFlipUtil.apply(startingPose));
              drive.setPose(AllianceFlipUtil.apply(startingPose));
            }),
        AutoBuilder.pathfindToPose(
            DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? endingPose
                : AllianceFlipUtil.apply(endingPose),
            pathConstraints));
  }
}
