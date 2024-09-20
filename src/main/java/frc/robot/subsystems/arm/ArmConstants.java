package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ArmConstants {

  public static final int ARM_CAN_ID;
  public static final double ARM_GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double ARM_MOMENT_OF_INERTIA;
  public static final DCMotor ARM_MOTOR_CONFIG;
  public static final double ARM_STOW_CONSTANT;
  public static final double ARM_INTAKE_CONSTANT;
  public static final double ARM_AMP_CONSTANT;
  public static final Rotation2d ARM_ABSOLUTE_ENCODER_OFFSET;
  public static final int ARM_ENCODER_ID;
  public static final double GOAL_TOLERANCE;
  public static final double ARM_LENGTH_METERS;
  public static final double ARM_MIN_ANGLE;
  public static final double ARM_MAX_ANGLE;
  public static final LoggedTunableNumber ARM_KP;
  public static final LoggedTunableNumber ARM_KD;
  public static final LoggedTunableNumber ARM_KS;
  public static final LoggedTunableNumber ARM_KG;
  public static final LoggedTunableNumber ARM_KV;
  public static final LoggedTunableNumber ARM_MAX_VELOCITY;
  public static final LoggedTunableNumber ARM_MAX_ACCELERATION;

  static {
    ARM_KP = new LoggedTunableNumber("Arm/KP");
    ARM_KD = new LoggedTunableNumber("Arm/KD");
    ARM_KS = new LoggedTunableNumber("Arm/KS");
    ARM_KG = new LoggedTunableNumber("Arm/KG");
    ARM_KV = new LoggedTunableNumber("Arm/KV");
    ARM_MAX_VELOCITY = new LoggedTunableNumber("Arm/MAX_VELOCITY");
    ARM_MAX_ACCELERATION = new LoggedTunableNumber("Arm/MAX_ACCELERATION");

    switch (Constants.ROBOT) {
      case ROBOT_KRAKEN_X60:
      case ROBOT_SIM:
      default:
        ARM_CAN_ID = 1;

        ARM_GEAR_RATIO = 1.0;

        CURRENT_LIMIT = 40.0;

        ARM_MOMENT_OF_INERTIA = 0.004;

        ARM_MOTOR_CONFIG = DCMotor.getKrakenX60(1);

        ARM_STOW_CONSTANT = 0.0;
        ARM_INTAKE_CONSTANT = 0.0;
        ARM_AMP_CONSTANT = 0.0;
        ARM_ABSOLUTE_ENCODER_OFFSET = new Rotation2d(0.0);
        ARM_ENCODER_ID = 0;
        GOAL_TOLERANCE = 0.0;
        ARM_LENGTH_METERS = 0.0;
        ARM_MIN_ANGLE = 0.0;
        ARM_MAX_ANGLE = 0.0;
        ARM_KP.initDefault(0.0);
        ARM_KD.initDefault(0.0);
        ARM_KS.initDefault(0.0);
        ARM_KG.initDefault(0.0);
        ARM_KV.initDefault(0.0);
        ARM_MAX_VELOCITY.initDefault(0.0);
        ARM_MAX_ACCELERATION.initDefault(0.0);
        break;
    }
  }
}
