// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.accelerator.AcceleratorIO;
import frc.robot.subsystems.accelerator.AcceleratorIOSim;
import frc.robot.subsystems.accelerator.AcceleratorIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.serializer.SerializerIOSim;
import frc.robot.subsystems.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TrackingMode;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Shooter shooter;
  private Hood hood;
  private Intake intake;
  private Serializer serializer;
  private Kicker kicker;
  private Accelerator accelerator;
  private Climber climber;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Tunable Numbers
  private final LoggedDashboardNumber autoDelay = new LoggedDashboardNumber("Auto Delay");

  // Note Tracking
  private static boolean isNoteTracking = false;

  // Dashboard Inputs
  private LoggedDashboardChooser<String> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case SNAPBACK:
          // Snapback, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          shooter = new Shooter(new ShooterIOTalonFX());
          hood = new Hood(new HoodIOTalonFX());
          intake = new Intake(new IntakeIOTalonFX());
          serializer = new Serializer(new SerializerIOTalonFX());
          kicker = new Kicker(new KickerIOTalonFX());
          accelerator = new Accelerator(new AcceleratorIOTalonFX());
          climber = new Climber(new ClimberIOTalonFX());
          vision = new Vision(RobotCameras.LIMELIGHT_CENTER);
          break;
        case ROBOT_2K24_TEST:
          // Test robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          break;

        case ROBOT_SIM:
          // Sim robot, instantiate physics sim IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          shooter = new Shooter(new ShooterIOSim());
          hood = new Hood(new HoodIOSim());
          intake = new Intake(new IntakeIOSim());
          serializer = new Serializer(new SerializerIOSim());
          kicker = new Kicker(new KickerIOSim());
          accelerator = new Accelerator(new AcceleratorIOSim());
          climber = new Climber(new ClimberIOSim());
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (hood == null) {
      hood = new Hood(new HoodIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (serializer == null) {
      serializer = new Serializer(new SerializerIO() {});
    }
    if (kicker == null) {
      kicker = new Kicker(new KickerIO() {});
    }
    if (accelerator == null) {
      accelerator = new Accelerator(new AcceleratorIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (vision == null) {
      vision = new Vision();
    }

    // Configure autobuilder
    AutoBuilder.configureHolonomic(
        RobotState::getRobotPose,
        RobotState::resetRobotPose,
        () -> DriveConstants.KINEMATICS.toChassisSpeeds(drive.getModuleStates()),
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            frc.robot.subsystems.drive.drive.DriveConstants.MAX_LINEAR_VELOCITY,
            frc.robot.subsystems.drive.drive.DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "LocalADStarAK/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("LocalADStarAK/Trajectory Setpoint", targetPose);
        });

    // Configure auto choices.
    autoChooser = new LoggedDashboardChooser<>("Auto Routines");
    for (String routine : AutoRoutines.autoList) {
      if (routine.equals(AutoRoutines.autoList.get(0))) {
        autoChooser.addDefaultOption(routine, routine);
      } else {
        autoChooser.addOption(routine, routine);
      }
    }

    // Configure RobotState
    new RobotState(
        drive::getRotation,
        drive::getYawVelocity,
        drive::getFieldRelativeVelocity,
        drive::getModulePositions,
        vision::getCameras,
        vision::getValidTarget,
        vision::getPrimaryVisionPoses,
        vision::getSecondaryVisionPoses,
        vision::getFrameTimestamps);

    // Configure the button bindings
    configureButtonBindings();

    // Configure shuffleboard
    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Hood Offset", RobotState::getHoodOffset)
        .withPosition(0, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Flywheel Offset", RobotState::getFlywheelOffset)
        .withPosition(0, 1)
        .withSize(1, 1);
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            vision,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper()));
    driver.start().onTrue(CompositeCommands.resetHeading());
    driver
        .rightTrigger()
        .whileTrue(CompositeCommands.getSourceFeedCommand(shooter, hood, accelerator, kicker));
    driver.leftTrigger().whileTrue(CompositeCommands.getOuttakeCommand(intake, serializer, kicker));
    driver
        .leftBumper()
        .whileTrue(CompositeCommands.getCollectCommand(intake, serializer))
        .onFalse(intake.retractIntake());
    driver
        .rightBumper()
        .whileTrue(CompositeCommands.getPosePrepShooterCommand(drive, hood, shooter, accelerator));
    driver
        .rightBumper()
        .and(() -> RobotState.shooterReady(hood, shooter))
        .whileTrue(
            Commands.waitSeconds(0.25)
                .andThen(CompositeCommands.getShootCommand(intake, serializer, kicker))
                .withTimeout(0.5));
    driver.b().whileTrue(CompositeCommands.getShootCommand(intake, serializer, kicker));
    driver.a().whileTrue(intake.singleActuation());

    operator
        .rightBumper()
        .whileTrue(CompositeCommands.getAmpFeedCommand(shooter, hood, accelerator, kicker));
    operator.y().whileTrue(CompositeCommands.increaseFlywheelVelocity());
    operator.a().whileTrue(CompositeCommands.decreaseFlywheelVelocity());
    operator.povUp().onTrue(climber.preClimb());
    operator.povDown().onTrue(climber.climbAutomatic());
    operator.back().onTrue(climber.zero());
    operator.leftBumper().onTrue(CompositeCommands.decreaseHoodAngle());
    operator.leftTrigger().onTrue(CompositeCommands.increaseHoodAngle());
    operator.start().onTrue(Commands.runOnce(() -> isNoteTracking = !isNoteTracking));
  }

  public Command getAutonomousCommand() {
    Map<String, Command> autoMap =
        AutoRoutines.getAutoList(drive, intake, serializer, kicker, TrackingMode.NOTES);
    return Commands.parallel(
        Commands.waitSeconds(autoDelay.get()).andThen(autoMap.get(autoChooser.get())),
        CompositeCommands.getPosePrepShooterCommand(drive, hood, shooter, accelerator));
  }
}
