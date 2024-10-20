package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraType;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeometryUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingDoubleTreeMap speakerShotAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  @Getter
  private static ControlData controlData =
      new ControlData(new Rotation2d(), 0.0, new Rotation2d(), false, false, false, false);

  @Getter @Setter private static double speakerFlywheelCompensation = 0.0;
  @Getter @Setter private static double speakerAngleCompensation = 0.0;

  private static SwerveDrivePoseEstimator poseEstimator;

  private static Rotation2d robotHeading;
  private static SwerveModulePosition[] modulePositions;

  static {
    // Units: radians
    speakerShotAngleMap.put(1.0051382994805276, Units.degreesToRadians(57.0));
    speakerShotAngleMap.put(1.4924089439984491, 0.84);
    speakerShotAngleMap.put(2.0188748058905883, 0.74);
    speakerShotAngleMap.put(2.494223768158363, 0.66);
    speakerShotAngleMap.put(2.997906851949344, 0.57);
    speakerShotAngleMap.put(3.481117210151285, 0.5);
    speakerShotAngleMap.put(3.992798130214426, 0.46);
    speakerShotAngleMap.put(4.590536757726377, 0.44);
    speakerShotAngleMap.put(4.9909464332643125, 0.42);
    speakerShotAngleMap.put(5.508818126964896, 0.4);
    speakerShotAngleMap.put(6.067253283488031, 0.37);

    // Units: seconds
    timeOfFlightMap.put(0.0, 0.0);

    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, new Rotation2d(), modulePositions, new Pose2d());
  }

  public RobotState() {}

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      Camera[] cameras,
      boolean hasNoteLocked,
      boolean hasNoteStaged,
      boolean isIntaking,
      boolean isClimbed) {

    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), robotHeading, modulePositions);
    long timestamp = NetworkTablesJNI.now();

    for (Camera camera : cameras) {
      if (camera.getCameraType() == CameraType.LIMELIGHT_3G
          || camera.getCameraType() == CameraType.LIMELIGHT_3) {
        double[] limelightHeadingData = {
          RobotState.getRobotPose().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
        };
        camera.getRobotHeadingPublisher().set(limelightHeadingData, timestamp);
      }

      if (camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && !GeometryUtil.isZero(camera.getSecondaryPose())) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        poseEstimator.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
        if (camera.getAverageDistance() <= 1.0) {
          double xyStddevSecondary =
              camera.getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camera.getAverageDistance(), 2.0)
                  / camera.getTotalTargets()
                  * camera.getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              camera.getSecondaryPose(),
              camera.getFrameTimestamp(),
              VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, Double.POSITIVE_INFINITY));
        }
      }
    }

    // Speaker Shot Calculations
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    double distanceToSpeaker =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(speakerPose);
    Translation2d effectiveSpeakerAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(robotFieldRelativeVelocity.times(timeOfFlightMap.get(distanceToSpeaker)));
    double effectiveDistanceToSpeaker = effectiveSpeakerAimingPose.getDistance(speakerPose);
    Rotation2d speakerRobotAngle =
        speakerPose
            .minus(effectiveSpeakerAimingPose)
            .getAngle()
            .minus(Rotation2d.fromDegrees(180.0 + 3.5));
    double speakerTangentialVelocity =
        -robotFieldRelativeVelocity.rotateBy(speakerRobotAngle.unaryMinus()).getY();
    double speakerRadialVelocity = speakerTangentialVelocity / effectiveDistanceToSpeaker;

    controlData =
        new ControlData(
            speakerRobotAngle,
            speakerRadialVelocity,
            new Rotation2d(speakerShotAngleMap.get(effectiveDistanceToSpeaker)),
            hasNoteLocked,
            hasNoteStaged,
            isIntaking,
            isClimbed);

    Logger.recordOutput(
        "RobotState/Pose Data/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/Pose Data/Effective Speaker Aiming Pose",
        new Pose2d(effectiveSpeakerAimingPose, new Rotation2d()));
    Logger.recordOutput(
        "RobotState/Pose Data/Effective Distance To Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "RobotState/Signal Data/Rio Bus Utilization",
        RobotController.getCANStatus().percentBusUtilization);
    Logger.recordOutput(
        "RobotState/Signal Data/CANivore Bus Utilization",
        CANBus.getStatus(DriveConstants.CANIVORE).BusUtilization);
    Logger.recordOutput(
        "RobotState/ControlData/Speaker Robot Angle", controlData.speakerRobotAngle());
    Logger.recordOutput("RobotState/ControlData/Speaker Arm Angle", controlData.speakerArmAngle());
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(robotHeading, modulePositions, pose);
  }

  public static record ControlData(
      Rotation2d speakerRobotAngle,
      double speakerRadialVelocity,
      Rotation2d speakerArmAngle,
      boolean hasNoteLocked,
      boolean hasNoteStaged,
      boolean isIntaking,
      boolean isClimbed) {}
}
