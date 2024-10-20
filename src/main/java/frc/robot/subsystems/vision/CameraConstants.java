package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

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
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  public static class ArducamOV9281 {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double AVERAGE_BEST_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  public static class RobotCameras {
    public static final Camera LEFT_CAMERA =
        new Camera(
            new CameraIOLimelight("left", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault().getDoubleArrayTopic("limelight-left").publish());
    public static final Camera RIGHT_CAMERA =
        new Camera(
            new CameraIOLimelight("right", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault().getDoubleArrayTopic("limelight-right").publish());
  }

  public static class ReplayCameras {
    public static final Camera LEFT_CAMERA =
        new Camera(
            new CameraIO() {},
            RobotCameras.LEFT_CAMERA.getName(),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault().getDoubleArrayTopic("limelight-left").publish());
    public static final Camera RIGHT_CAMERA =
        new Camera(
            new CameraIO() {},
            RobotCameras.RIGHT_CAMERA.getName(),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault().getDoubleArrayTopic("limelight-right").publish());
  }
}
