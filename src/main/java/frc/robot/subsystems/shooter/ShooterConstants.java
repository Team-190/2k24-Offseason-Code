package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/**
 * The ShooterConstants class
 *
 * @author Sriaditya Vaddadi
 */
public class ShooterConstants {

  public static final int TOP_CAN_ID;
  public static final int BOTTOM_CAN_ID;
  public static final double TOP_GEAR_RATIO;
  public static final double BOTTOM_GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double TOP_MOMENT_OF_INERTIA;
  public static final double BOTTOM_MOMENT_OF_INERTIA;
  public static final DCMotor TOP_MOTOR_CONFIG;
  public static final DCMotor BOTTOM_MOTOR_CONFIG;
  public static final LoggedTunableNumber KP;
  public static final LoggedTunableNumber KD;
  public static final LoggedTunableNumber KS;
  public static final LoggedTunableNumber KV;
  public static final LoggedTunableNumber KA;
  public static final LoggedTunableNumber MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
  public static final LoggedTunableNumber AMP_SPEED;
  public static final LoggedTunableNumber SUBWOOFER_SPEED;
  public static final double PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND;

  static {
    KP = new LoggedTunableNumber("Shooter/kP");
    KD = new LoggedTunableNumber("Shooter/kD");
    KS = new LoggedTunableNumber("Shooter/kS");
    KV = new LoggedTunableNumber("Shooter/kV");
    KA = new LoggedTunableNumber("Shooter/kA");
    MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
        new LoggedTunableNumber("Shooter/Max Acceleration");
    AMP_SPEED = new LoggedTunableNumber("Shooter/Amp Speed");
    SUBWOOFER_SPEED = new LoggedTunableNumber("Shooter/Subwoofer Speed");

    switch (Constants.ROBOT) {
      default:
        TOP_CAN_ID = 14;
        BOTTOM_CAN_ID = 15;
        TOP_GEAR_RATIO = 1.0;
        BOTTOM_GEAR_RATIO = 1.0;
        CURRENT_LIMIT = 40.0;
        TOP_MOMENT_OF_INERTIA = 0.004;
        BOTTOM_MOMENT_OF_INERTIA = 0.004;
        TOP_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        BOTTOM_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        KP.initDefault(0.325);
        KD.initDefault(0.0);
        KS.initDefault(0.0);
        KV.initDefault(0.019005);
        KA.initDefault(0.0067707);
        MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.initDefault(10.0);
        PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND = 1.0;
        AMP_SPEED.initDefault(300.0);
        SUBWOOFER_SPEED.initDefault(1000.0);
        break;
    }
  }
}
