package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraConstants {
  public static final double BLINK_TIME = 0.067;

  public static class Limelight3Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  public static class Limelight3GConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.0026;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.0015;
  }

  public static class ArducamOV9281 {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double AVERAGE_BEST_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  public static class RobotCameras {
    public static final Camera LIMELIGHT_CENTER =
        new Camera(
            new CameraIOLimelight("lime", CameraType.LIMELIGHT_3G, CameraMode.APRILTAGS),
            0.0,
            0.0,
            0.0,
            0.0);
    public static final Camera LIMELIGHT_LEFT_ARDUCAM =
        new Camera(
            new CameraIOPhotonVision(
                "LL_Left_Arducam",
                CameraType.ARDUCAM_OV9281,
                FieldConstants.AprilTagConstants.FIELD_LAYOUT_2024,
                new Transform3d(
                    0.117767,
                    -0.349005,
                    -0.168195,
                    new Rotation3d(
                        0.0,
                        Units.degreesToRadians(-45.0),
                        Units.degreesToRadians(-90 - 38.359636))),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                PoseStrategy.AVERAGE_BEST_TARGETS,
                CameraMode.APRILTAGS),
            ArducamOV9281.HORIZONTAL_FOV,
            ArducamOV9281.VERTICAL_FOV,
            ArducamOV9281.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            ArducamOV9281.AVERAGE_BEST_XY_STANDARD_DEVIATION_COEFFICIENT);
    public static final Camera LIMELIGHT_RIGHT_ARDUCAM =
        new Camera(
            new CameraIOPhotonVision(
                "LL_Right_Arducam",
                CameraType.ARDUCAM_OV9281,
                FieldConstants.AprilTagConstants.FIELD_LAYOUT_2024,
                new Transform3d(
                    -0.117767,
                    -0.349005,
                    -0.168195,
                    new Rotation3d(
                        0.0,
                        Units.degreesToRadians(-45.0),
                        Units.degreesToRadians(-90 + 38.359636))),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                PoseStrategy.AVERAGE_BEST_TARGETS,
                CameraMode.APRILTAGS),
            ArducamOV9281.HORIZONTAL_FOV,
            ArducamOV9281.VERTICAL_FOV,
            ArducamOV9281.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            ArducamOV9281.AVERAGE_BEST_XY_STANDARD_DEVIATION_COEFFICIENT);
  }
}
