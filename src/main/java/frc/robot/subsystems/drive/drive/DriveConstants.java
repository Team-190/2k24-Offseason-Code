package frc.robot.subsystems.drive.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public final class DriveConstants {
  public static final LoggedTunableNumber AUTO_AIM_KP = new LoggedTunableNumber("Auto Aim/Kp");
  public static final LoggedTunableNumber AUTO_AIM_KD = new LoggedTunableNumber("Auto Aim/Kd");
  public static final LoggedTunableNumber AUTO_AIM_X_VEL_MAX =
      new LoggedTunableNumber("Auto Aim/Max X Velocity");
  public static final LoggedTunableNumber AUTO_AIM_X_VEL_MIN =
      new LoggedTunableNumber("Auto Aim/Min X Velocity");
  public static final LoggedTunableNumber AUTO_AIM_X_VEL_RANGE =
      new LoggedTunableNumber("Auto Aim/X Velocity Range");
  public static final LoggedTunableNumber AUTO_AIM_FIELD_VELOCITY_DEADBAND =
      new LoggedTunableNumber("Auto Aim/Field Velocity Deadband");

  public static final double WHEEL_RADIUS;
  public static final double TRACK_WIDTH_X;
  public static final double TRACK_WIDTH_Y;
  public static final double MAX_LINEAR_VELOCITY;
  public static final double MAX_ANGULAR_VELOCITY;
  public static final double DRIVE_BASE_RADIUS;
  public static final SwerveDriveKinematics KINEMATICS;
  public static final String CANIVORE;
  public static final int PIGEON_2_DEVICE_ID;
  public static final Matrix<N3, N1> ODOMETRY_STANDARD_DEVIATIONS;
  public static final double DRIVER_DEADBAND;
  public static final Lock ODOMETRY_LOCK;

  static {
    AUTO_AIM_X_VEL_MAX.initDefault(1.5);
    AUTO_AIM_X_VEL_MIN.initDefault(0.5);
    AUTO_AIM_X_VEL_RANGE.initDefault(0.5);
    AUTO_AIM_FIELD_VELOCITY_DEADBAND.initDefault(0.5);

    WHEEL_RADIUS = Units.inchesToMeters(2.0);
    TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    MAX_LINEAR_VELOCITY = Units.feetToMeters(17.5);
    DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
    MAX_ANGULAR_VELOCITY = MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS;
    KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d[] {
              new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
              new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
              new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
              new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
            });
    CANIVORE = "drive";
    PIGEON_2_DEVICE_ID = 1;
    ODOMETRY_STANDARD_DEVIATIONS = VecBuilder.fill(0.0, 0.0, 0.0);
    DRIVER_DEADBAND = 0.1;
    ODOMETRY_LOCK = new ReentrantLock();
    switch (Constants.ROBOT) {
      case SNAPBACK:
        AUTO_AIM_KP.initDefault(6.0);
        AUTO_AIM_KD.initDefault(0.002);
        break;
      case ROBOT_2K24_TEST:
        AUTO_AIM_KP.initDefault(6.0);
        AUTO_AIM_KD.initDefault(0.002);
        break;
      case ROBOT_SIM:
        AUTO_AIM_KP.initDefault(7);
        AUTO_AIM_KD.initDefault(0.125);
      default:
        break;
    }
  }
}
