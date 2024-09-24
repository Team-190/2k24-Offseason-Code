package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public final class AutoRoutines {
  public static final Command none() {
    return Commands.none();
  }

  public static final Command getInitialChoreoCommand(Drive drive, String trajectory) {
    ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(trajectory);
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red) {
                RobotState.resetRobotPose(choreoTrajectory.getFlippedInitialPose());
              } else {
                RobotState.resetRobotPose(choreoTrajectory.getInitialPose());
              }
            }),
        Choreo.choreoSwerveCommand(
            choreoTrajectory,
            RobotState::getRobotPose,
            new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get()),
            new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get()),
            new PIDController(
                DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get()),
            (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red,
            drive));
  }

  public static final Command getChoreoCommand(Drive drive, String trajectory) {
    return Choreo.choreoSwerveCommand(
        Choreo.getTrajectory(trajectory),
        RobotState::getRobotPose,
        new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get()),
        new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get()),
        new PIDController(
            DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get()),
        (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        drive);
  }

  public static final Command test(Drive drive) {
    return getInitialChoreoCommand(drive, "Test");
  }

  public static final Command CenterAuto(Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        getInitialChoreoCommand(drive, "Center_Path_1"),
        CompositeCommands.shootSpeaker(intake, arm, shooter),
        Commands.race(
            getChoreoCommand(drive, "Center_Path_2"), CompositeCommands.collect(intake, arm)),
        getChoreoCommand(drive, "Center_Path_3"),
        CompositeCommands.shootSpeaker(intake, arm, shooter),
        Commands.race(
            getChoreoCommand(drive, "Center_Path_4"), CompositeCommands.collect(intake, arm)),
        getChoreoCommand(drive, "Center_Path_5"),
        CompositeCommands.shootSpeaker(intake, arm, shooter));
  }
}
