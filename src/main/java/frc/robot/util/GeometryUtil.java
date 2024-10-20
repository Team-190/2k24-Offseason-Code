package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class GeometryUtil {
  public static final boolean isZero(Pose2d pose) {
    return pose.getX() == 0.0 && pose.getY() == 0.0 && pose.getRotation().getDegrees() == 0.0;
  }
}
