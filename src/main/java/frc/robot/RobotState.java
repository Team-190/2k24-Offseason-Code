package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraType;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingDoubleTreeMap shooterSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  @Getter
  private static StateCache stateCache =
      new StateCache(new Rotation2d(), 0.0, 0.0, new Rotation2d());

  @Getter @Setter private static double flywheelOffset = 0.0;
  @Getter @Setter private static double hoodOffset = 0.0;

  private static SwerveDrivePoseEstimator poseEstimator;

  private static Supplier<Rotation2d> robotHeadingSupplier;
  private static DoubleSupplier robotYawVelocitySupplier;
  private static Supplier<Translation2d> robotFieldRelativeVelocitySupplier;
  private static Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private static Supplier<Camera[]> camerasSupplier;
  private static BooleanSupplier targetAquiredSupplier;
  private static Supplier<Optional<Pose3d>[]> visionPrimaryPosesSupplier;
  private static Supplier<Optional<Pose3d>[]> visionSecondaryPosesSupplier;
  private static Supplier<double[]> visionFrameTimestampSupplier;

  static {
    // Units: radians per second
    shooterSpeedMap.put(2.16, 800.0);
    shooterSpeedMap.put(2.45, 800.0);
    shooterSpeedMap.put(2.69, 800.0);
    shooterAngleMap.put(2.84, 800.0);
    shooterSpeedMap.put(3.19, 800.0);
    shooterSpeedMap.put(3.52, 800.0);
    shooterSpeedMap.put(3.85, 900.0);
    shooterSpeedMap.put(4.29, 900.0);

    // Units: radians
    shooterAngleMap.put(2.16, 0.05);
    shooterAngleMap.put(2.45, 0.05);
    shooterAngleMap.put(2.69, 0.16);
    shooterAngleMap.put(2.84, 0.32);
    shooterAngleMap.put(3.19, 0.39);
    shooterAngleMap.put(3.52, 0.45);
    shooterAngleMap.put(3.85, 0.44);
    shooterAngleMap.put(4.29, 0.45);

    // Units: seconds
    timeOfFlightMap.put(2.50, (4.42 - 4.24));
    timeOfFlightMap.put(2.75, (2.56 - 2.33));
    timeOfFlightMap.put(3.00, (3.43 - 3.18));
    timeOfFlightMap.put(3.25, (3.20 - 2.94));
    timeOfFlightMap.put(3.50, (2.64 - 2.42));
    timeOfFlightMap.put(4.0, (2.60 - 2.32));
  }

  public RobotState(
      Supplier<Rotation2d> robotHeadingSupplier,
      DoubleSupplier robotYawVelocitySupplier,
      Supplier<Translation2d> robotFieldRelativeVelocitySupplier,
      Supplier<SwerveModulePosition[]> modulePositionSupplier,
      Supplier<Camera[]> camerasSupplier,
      BooleanSupplier targetAquiredSupplier,
      Supplier<Optional<Pose3d>[]> visionPrimaryPosesSupplier,
      Supplier<Optional<Pose3d>[]> visionSecondaryPosesSupplier,
      Supplier<double[]> visionFrameTimestampSupplier) {
    RobotState.robotHeadingSupplier = robotHeadingSupplier;
    RobotState.robotYawVelocitySupplier = robotYawVelocitySupplier;
    RobotState.robotFieldRelativeVelocitySupplier = robotFieldRelativeVelocitySupplier;
    RobotState.modulePositionSupplier = modulePositionSupplier;
    RobotState.camerasSupplier = camerasSupplier;
    RobotState.targetAquiredSupplier = targetAquiredSupplier;
    RobotState.visionPrimaryPosesSupplier = visionPrimaryPosesSupplier;
    RobotState.visionSecondaryPosesSupplier = visionSecondaryPosesSupplier;
    RobotState.visionFrameTimestampSupplier = visionFrameTimestampSupplier;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            robotHeadingSupplier.get(),
            modulePositionSupplier.get(),
            new Pose2d(),
            DriveConstants.ODOMETRY_STANDARD_DEVIATIONS,
            VecBuilder.fill(0.0, 0.0, 0.0));
  }

  public static final void periodic() {
    Camera[] cameras = camerasSupplier.get();
    for (Camera camera : cameras) {
      if (camera.getCameraType() == CameraType.LIMELIGHT_3G
          || camera.getCameraType() == CameraType.LIMELIGHT_3) {
        LimelightHelpers.SetRobotOrientation(
            camera.getName(), robotHeadingSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
      }
    }
    addVisionMeasurement();
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), robotHeadingSupplier.get(), modulePositionSupplier.get());

    Logger.recordOutput("RobotState/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput("RobotState/Target Aquired", targetAquiredSupplier.getAsBoolean());
  }

  private static final void addVisionMeasurement() {
    if (targetAquiredSupplier.getAsBoolean()
        && robotYawVelocitySupplier.getAsDouble() < Units.degreesToRadians(720.0)) {
      for (int i = 0; i < visionPrimaryPosesSupplier.get().length; i++) {
        if (visionSecondaryPosesSupplier.get()[i].isPresent()) {
          double xyStddev =
              camerasSupplier.get()[i].getPrimaryXYStandardDeviationCoefficient()
                  * Math.pow(camerasSupplier.get()[i].getAverageDistance(), 2.0)
                  / camerasSupplier.get()[i].getTotalTargets()
                  * camerasSupplier.get()[i].getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              visionPrimaryPosesSupplier.get()[i].get().toPose2d(),
              visionFrameTimestampSupplier.get()[i],
              VecBuilder.fill(xyStddev, xyStddev, Double.POSITIVE_INFINITY));
        }
      }
      for (int i = 0; i < visionSecondaryPosesSupplier.get().length; i++) {
        if (visionSecondaryPosesSupplier.get()[i].isPresent()) {
          double xyStddev =
              camerasSupplier.get()[i].getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camerasSupplier.get()[i].getAverageDistance(), 2.0)
                  / camerasSupplier.get()[i].getTotalTargets()
                  * camerasSupplier.get()[i].getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              visionSecondaryPosesSupplier.get()[i].get().toPose2d(),
              visionFrameTimestampSupplier.get()[i],
              VecBuilder.fill(xyStddev, xyStddev, Double.POSITIVE_INFINITY));
        }
      }
    }

    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    double distanceToSpeaker =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(speakerPose);
    Translation2d effectiveAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(
                robotFieldRelativeVelocitySupplier
                    .get()
                    .times(timeOfFlightMap.get(distanceToSpeaker)));
    double effectiveDistanceToSpeaker = effectiveAimingPose.getDistance(speakerPose);

    Rotation2d setpointAngle = speakerPose.minus(effectiveAimingPose).getAngle();
    double tangentialVelocity =
        -robotFieldRelativeVelocitySupplier.get().rotateBy(setpointAngle.unaryMinus()).getY();
    double radialVelocity = tangentialVelocity / effectiveDistanceToSpeaker;
    stateCache =
        new StateCache(
            setpointAngle,
            radialVelocity,
            shooterSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)));

    Logger.recordOutput("RobotState/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput("RobotState/StateCache/Robot Angle Setpoint", setpointAngle);
    Logger.recordOutput(
        "RobotState/StateCache/Effective Distance to Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "RobotState/StateCache/Effective Aiming Pose",
        new Pose2d(effectiveAimingPose, new Rotation2d()));
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(robotHeadingSupplier.get(), modulePositionSupplier.get(), pose);
  }

  public static Rotation2d getTargetGyroOffset(Pose2d targetPose) {
    return Rotation2d.fromRadians(
        Math.atan2(
            targetPose.getY() - getRobotPose().getY(), targetPose.getX() - getRobotPose().getX()));
  }

  public static boolean shooterReady(Hood hood, Shooter shooter) {
    return shooter.atGoal() && hood.atGoal();
  }

  public static record StateCache(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}
}
