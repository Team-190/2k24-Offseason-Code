package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberConstants {
  public static final int CLIMBER_CAN_ID;
  public static final double CURRENT_LIMIT;
  public static final DCMotor CLIMBER_MOTOR;
  public static final double GEAR_RATIO;
  public static final double DRUM_RADIUS;
  public static final DCMotor DC_MOTOR_CONFIG;
  public static final double CARRIAGE_MASS;
  public static final double STARTING_HEIGHT;
  public static final double MIN_HEIGHT;
  public static final double MAX_HEIGHT;
  public static final Rotation2d RELEASE_POSITION;
  public static final Rotation2d CLIMB_POSITION;
  public static final boolean GRAVITY;
  public static final double HOLD_VOLTAGE;

  static {
    switch (Constants.ROBOT) {
      case WHIPLASH:
      case ROBOT_SIM:
      default:
        CLIMBER_CAN_ID = 9;
        CURRENT_LIMIT = 40.0;
        CLIMBER_MOTOR = DCMotor.getKrakenX60(1);
        GEAR_RATIO = 21.66666667;
        DRUM_RADIUS = Units.inchesToMeters(.75 / 2);
        DC_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        CARRIAGE_MASS = 0.1; // TODO: Update Values
        STARTING_HEIGHT = 0;
        MIN_HEIGHT = 0; // TODO: Update Values
        MAX_HEIGHT = Units.inchesToMeters(29.043 - 7.043); // TODO: Update Values
        GRAVITY = false;
        RELEASE_POSITION = Rotation2d.fromRadians(3.0);
        CLIMB_POSITION = Rotation2d.fromRadians(53);
        HOLD_VOLTAGE = 0.5;
        break;
    }
  }
}
