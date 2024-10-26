package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public final class AutoRoutines {
  public static final Command none() {
    return Commands.none();
  }

  public static final Command resetToInitialHeading(Drive drive, String trajectory) {
    ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(trajectory);
    return Commands.runOnce(
        () -> RobotState.resetRobotPose(AllianceFlipUtil.apply(choreoTrajectory.getInitialPose())));
  }

  /**
   * This function generates a command sequence for autonomous driving using the Choreo library. The
   * command sequence includes executing a trajectory from the Choreo library, and running the
   * trajectory using a Swerve drive.
   *
   * @param drive The drive subsystem instance. This is used to control the robot's movement.
   * @param trajectory The name of the trajectory to be executed. The trajectory is defined in the
   *     Choreo library.
   * @return A command sequence for autonomous driving. This command sequence can be used to control
   *     the robot during autonomous mode.
   */
  public static final Command getChoreoCommand(Drive drive, String trajectory) {
    PIDController xFeedback =
        new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get());
    PIDController yFeedback =
        new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get());
    PIDController thetaFeedback =
        new PIDController(
            DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get());
    thetaFeedback.enableContinuousInput(-Math.PI, Math.PI);
    ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(trajectory);
    return Choreo.choreoSwerveCommand(
        choreoTrajectory,
        RobotState::getRobotPose,
        xFeedback,
        yFeedback,
        thetaFeedback,
        (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
        () -> AllianceFlipUtil.shouldFlip(),
        drive);
  }

  public static final Command getBoatBattleAmpAuto(
      Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        resetToInitialHeading(drive, "Auto_1_Amp"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter),
        Commands.race(
            getChoreoCommand(drive, "Auto_1_Amp"), CompositeCommands.collect(intake, arm)),
        getChoreoCommand(drive, "Auto_2_Amp"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter));
  }

  public static final Command getBoatBattleSourceAuto(
      Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        resetToInitialHeading(drive, "Auto_1_Source"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter),
        Commands.race(
            getChoreoCommand(drive, "Auto_1_Source"), CompositeCommands.collect(intake, arm)),
        getChoreoCommand(drive, "Auto_2_Source"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter));
  }

  public static final Command get4PieceAuto(Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        resetToInitialHeading(drive, "Auto_1_4"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter),
        Commands.race(getChoreoCommand(drive, "Auto_1_4"), CompositeCommands.collect(intake, arm)),
        Commands.race(
            DriveCommands.aimTowardSpeaker(drive),
            CompositeCommands.shootSpeakerAuto(drive, intake, arm, shooter)),
        Commands.race(getChoreoCommand(drive, "Auto_2_4"), CompositeCommands.collect(intake, arm)),
        Commands.race(
            DriveCommands.aimTowardSpeaker(drive),
            CompositeCommands.shootSpeakerAuto(drive, intake, arm, shooter)),
        getChoreoCommand(drive, "Auto_3_4"));
    // Commands.race(
    //     DriveCommands.aimTowardSpeaker(drive),
    //     CompositeCommands.shootSpeakerAuto(drive, intake, arm, shooter)));
  }
}
