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

  /**
   * This function generates a command sequence for autonomous driving using Choreo library. The
   * command sequence includes resetting the robot's pose based on the alliance color, executing a
   * trajectory from the Choreo library, and running the trajectory using a Swerve drive.
   *
   * @param drive The drive subsystem instance.
   * @param trajectory The name of the trajectory to be executed.
   * @return A command sequence for autonomous driving.
   */
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

  public static final Command getBoatBattleAmpAuto(
      Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        CompositeCommands.shootSubwoofer(intake, arm, shooter),
        Commands.race(getInitialChoreoCommand(drive, "Auto_1_Amp"), intake.intake()),
        getChoreoCommand(drive, "Auto_2_Amp"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter));
  }

  public static final Command getBoatBattleSourceAuto(
      Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        CompositeCommands.shootSubwoofer(intake, arm, shooter),
        Commands.race(getInitialChoreoCommand(drive, "Auto_1_Source"), intake.intake()),
        getChoreoCommand(drive, "Auto_2_Source"),
        CompositeCommands.shootSubwoofer(intake, arm, shooter));
  }
}
