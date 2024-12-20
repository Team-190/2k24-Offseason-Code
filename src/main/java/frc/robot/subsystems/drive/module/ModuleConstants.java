package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Builder;

public class ModuleConstants {
  public static final LoggedTunableNumber WHEEL_RADIUS;
  public static final LoggedTunableNumber DRIVE_KP;
  public static final LoggedTunableNumber DRIVE_KD;
  public static final LoggedTunableNumber DRIVE_KS;
  public static final LoggedTunableNumber DRIVE_KV;
  public static final LoggedTunableNumber TURN_KP;
  public static final LoggedTunableNumber TURN_KD;

  public static final ModuleConfiguration FRONT_LEFT;
  public static final ModuleConfiguration FRONT_RIGHT;
  public static final ModuleConfiguration REAR_LEFT;
  public static final ModuleConfiguration REAR_RIGHT;

  public static final double ODOMETRY_FREQUENCY;
  public static final double OUT_OF_SYNC_THRESHOLD;

  public static final double DRIVE_GEAR_RATIO;
  public static final double TURN_GEAR_RATIO;

  public static final int DRIVE_CURRENT_LIMIT;
  public static final int TURN_CURRENT_LIMIT;

  public static final double DRIVE_MOMENT_OF_INERTIA;
  public static final double TURN_MOMENT_OF_INERTIA;

  public static final DCMotor DRIVE_MOTOR_CONFIG;
  public static final DCMotor TURN_MOTOR_CONFIG;

  static {
    WHEEL_RADIUS = new LoggedTunableNumber("Drive/Wheel Radius");
    DRIVE_KP = new LoggedTunableNumber("Drive/Drive kP");
    DRIVE_KD = new LoggedTunableNumber("Drive/Drive_kD");
    DRIVE_KS = new LoggedTunableNumber("Drive/Drive kS");
    DRIVE_KV = new LoggedTunableNumber("Drive/Drive kV");
    TURN_KP = new LoggedTunableNumber("Drive/Turn kP");
    TURN_KD = new LoggedTunableNumber("Drive/Turn kD");

    switch (Constants.ROBOT) {
      case WHIPLASH:
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KP.initDefault(2.0);
        DRIVE_KD.initDefault(0.0);
        DRIVE_KS.initDefault(0.21831);
        DRIVE_KV.initDefault(0.095366);
        TURN_KP.initDefault(80.0);
        TURN_KD.initDefault(0.8);

        FRONT_LEFT = new ModuleConfiguration(1, 2, 20, Rotation2d.fromRadians(2.380738182798515));
        FRONT_RIGHT =
            new ModuleConfiguration(
                3, 4, 21, Rotation2d.fromRadians(-0.7240389318820226 + Math.PI));
        REAR_LEFT = new ModuleConfiguration(5, 6, 22, Rotation2d.fromRadians(1.6060778849162662));
        REAR_RIGHT = new ModuleConfiguration(7, 8, 23, Rotation2d.fromRadians(2.6614566669815876));

        ODOMETRY_FREQUENCY = 250.0;
        OUT_OF_SYNC_THRESHOLD = Units.degreesToRadians(30.0);

        DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
        TURN_GEAR_RATIO = 18.75;

        DRIVE_CURRENT_LIMIT = 40;
        TURN_CURRENT_LIMIT = 30;

        DRIVE_MOMENT_OF_INERTIA = 0.025;
        TURN_MOMENT_OF_INERTIA = 0.004;

        DRIVE_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        TURN_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        break;
      case ROBOT_SIM:
        WHEEL_RADIUS.initDefault(Units.inchesToMeters(2.0));
        DRIVE_KP.initDefault(0.05);
        DRIVE_KD.initDefault(0.0);
        DRIVE_KS.initDefault(0.0);
        DRIVE_KV.initDefault(0.10108);
        TURN_KP.initDefault(10.0);
        TURN_KD.initDefault(0.0);

        FRONT_LEFT = null;
        FRONT_RIGHT = null;
        REAR_LEFT = null;
        REAR_RIGHT = null;

        ODOMETRY_FREQUENCY = 250.0;
        OUT_OF_SYNC_THRESHOLD = Units.degreesToRadians(30.0);

        DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
        TURN_GEAR_RATIO = 150.0 / 7.0;

        DRIVE_CURRENT_LIMIT = 40;
        TURN_CURRENT_LIMIT = 30;

        DRIVE_MOMENT_OF_INERTIA = 0.025;
        TURN_MOMENT_OF_INERTIA = 0.004;

        DRIVE_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        TURN_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        break;
      default:
        WHEEL_RADIUS.initDefault(2.0);
        DRIVE_KP.initDefault(0.0);
        DRIVE_KD.initDefault(0.0);
        DRIVE_KS.initDefault(0.0);
        DRIVE_KV.initDefault(0.0);
        TURN_KP.initDefault(10.0);
        TURN_KD.initDefault(0.0);

        FRONT_LEFT = null;
        FRONT_RIGHT = null;
        REAR_LEFT = null;
        REAR_RIGHT = null;

        ODOMETRY_FREQUENCY = 250.0;
        OUT_OF_SYNC_THRESHOLD = Units.degreesToRadians(30.0);

        DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
        TURN_GEAR_RATIO = 150.0 / 7.0;

        DRIVE_CURRENT_LIMIT = 40;
        TURN_CURRENT_LIMIT = 30;

        DRIVE_MOMENT_OF_INERTIA = 0.025;
        TURN_MOMENT_OF_INERTIA = 0.004;

        DRIVE_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        TURN_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        break;
    }
  }

  @Builder
  public record ModuleConfiguration(
      int drive, int turn, int encoder, Rotation2d absoluteEncoderOffset) {}
}
