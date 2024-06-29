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

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TrackingMode;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static final Command joystickDrive(
      Drive drive,
      Vision aprilTagVision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier aprilTagTracking) {

    @SuppressWarnings({"resource"})
    PIDController aimController =
        new PIDController(
            frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_KP.get(),
            0,
            frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_KD.get(),
            Constants.LOOP_PERIOD_SECS);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  DriveConstants.DRIVER_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DRIVER_DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          if (aprilTagTracking.getAsBoolean()) {
            linearMagnitude =
                Math.min(linearMagnitude, 0.33); // change this to smaller for shoot on the move.
          }

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Configure PID
          aimController.setP(DriveConstants.AUTO_AIM_KP.get());
          aimController.setD(DriveConstants.AUTO_AIM_KD.get());

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          Optional<Rotation2d> targetGyroAngle = Optional.empty();
          Rotation2d measuredGyroAngle = drive.getRotation();
          double feedForwardRadialVelocity = 0.0;

          double robotRelativeXVel = linearVelocity.getX() * DriveConstants.MAX_LINEAR_VELOCITY;
          double robotRelativeYVel = linearVelocity.getY() * DriveConstants.MAX_LINEAR_VELOCITY;

          Pose2d visionPose = RobotState.getRobotPose();
          measuredGyroAngle = visionPose.getRotation();
          targetGyroAngle = Optional.of(RobotState.getStateCache().robotAngle());
          feedForwardRadialVelocity = RobotState.getStateCache().radialVelocity();
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  robotRelativeXVel,
                  robotRelativeYVel,
                  (aprilTagTracking.getAsBoolean()) && targetGyroAngle.isPresent()
                      ? feedForwardRadialVelocity
                          + aimController.calculate(
                              measuredGyroAngle.getRadians(), targetGyroAngle.get().getRadians())
                      : omega * DriveConstants.MAX_ANGULAR_VELOCITY,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());

          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static final Command aimTowardsTarget(Drive drive) {
    @SuppressWarnings({"resource"})
    PIDController aimController =
        new PIDController(
            frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_KP.get(),
            0,
            frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_KD.get(),
            Constants.LOOP_PERIOD_SECS);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    aimController.setTolerance(0.017);

    return Commands.run(
            () -> {
              // Configure PID
              aimController.setP(DriveConstants.AUTO_AIM_KP.get());
              aimController.setD(DriveConstants.AUTO_AIM_KD.get());

              // Get robot relative vel
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              Optional<Rotation2d> targetGyroAngle = Optional.empty();
              Rotation2d measuredGyroAngle = drive.getRotation();
              double feedForwardRadialVelocity = 0.0;

              Pose2d visionPose = RobotState.getRobotPose();
              measuredGyroAngle = visionPose.getRotation();
              targetGyroAngle = Optional.of(RobotState.getStateCache().robotAngle());
              feedForwardRadialVelocity = RobotState.getStateCache().radialVelocity();
              ChassisSpeeds chassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      0,
                      0,
                      feedForwardRadialVelocity
                          + (targetGyroAngle.isPresent()
                              ? aimController.calculate(
                                  measuredGyroAngle.getRadians(),
                                  targetGyroAngle.get().getRadians())
                              : 0),
                      isFlipped
                          ? RobotState.getRobotPose().getRotation().plus(new Rotation2d(Math.PI))
                          : RobotState.getRobotPose().getRotation());

              // Convert to field relative speeds & send command
              drive.runVelocity(chassisSpeeds);
            },
            drive)
        .until(() -> aimController.atSetpoint())
        .finallyDo(
            () -> {
              drive.stop();
              aimController.reset();
            });
  }

  public static final Command moveTowardsTarget(
      Drive drive, double blueXCoord, Pose2d targetPose, TrackingMode targetType) {

    @SuppressWarnings({"resource"})
    PIDController aimController =
        new PIDController(
            frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_KP.get(),
            0,
            frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_KD.get(),
            Constants.LOOP_PERIOD_SECS);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    DoubleSupplier targetXCoord =
        () -> {
          boolean isRed =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get().equals(Alliance.Red);
          return isRed ? FieldConstants.fieldLength - blueXCoord : blueXCoord;
        };

    return Commands.run(
            () -> {
              // Configure PID
              // Configure PID
              aimController.setP(DriveConstants.AUTO_AIM_KP.get());
              aimController.setD(DriveConstants.AUTO_AIM_KD.get());

              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get().equals(Alliance.Red);

              // Convert to field relative speeds & send command
              Rotation2d targetGyroOffset = RobotState.getTargetGyroOffset(targetPose);
              double distanceT =
                  MathUtil.clamp(
                      Math.abs(RobotState.getRobotPose().getX() - targetXCoord.getAsDouble())
                          / DriveConstants.AUTO_AIM_X_VEL_RANGE.get(),
                      0.0,
                      1.0);
              double speed =
                  MathUtil.interpolate(
                      frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_X_VEL_MIN.get(),
                      frc.robot.subsystems.drive.drive.DriveConstants.AUTO_AIM_X_VEL_MAX.get(),
                      distanceT);
              drive.runVelocity(
                  new ChassisSpeeds(
                      targetType.equals(TrackingMode.APRILTAGS) ? speed : -speed,
                      0,
                      aimController.calculate(
                          RobotState.getRobotPose().getRotation().getRadians(),
                          targetPose
                              .getRotation()
                              .plus(
                                  targetType.equals(TrackingMode.APRILTAGS)
                                      ? isRed
                                          ? targetGyroOffset
                                          : targetGyroOffset.plus(Rotation2d.fromRadians(Math.PI))
                                      : isRed
                                          ? targetGyroOffset.plus(Rotation2d.fromRadians(Math.PI))
                                          : targetGyroOffset)
                              .getRadians())));
            },
            drive)
        .until(
            () -> {
              boolean endAboveTargetXCoord;
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get().equals(Alliance.Red);
              if (isRed) {
                endAboveTargetXCoord = targetType.equals(TrackingMode.APRILTAGS);
              } else {
                endAboveTargetXCoord = targetType.equals(TrackingMode.NOTES);
              }
              if (endAboveTargetXCoord) {
                return RobotState.getRobotPose().getX() > targetXCoord.getAsDouble();
              } else {
                return RobotState.getRobotPose().getX() < targetXCoord.getAsDouble();
              }
            })
        .finallyDo(() -> drive.stop());
  }

  public static final Command runSysIdQuasistatic(Drive drive, Direction direction) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> drive.runCharacterizationVolts(volts.in(Volts)), null, drive))
        .quasistatic(direction);
  }

  public static final Command runSysIdDynamic(Drive drive, Direction direction) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> drive.runCharacterizationVolts(volts.in(Volts)), null, drive))
        .dynamic(direction);
  }
}
