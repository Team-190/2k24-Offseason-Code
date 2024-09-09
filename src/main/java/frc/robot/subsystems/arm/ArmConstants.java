package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class ArmConstants {

    public static final int ARM_CAN_ID;
    public static final double ARM_GEAR_RATIO;
    public static final double CURRENT_LIMIT;
    public static final double ARM_MOMENT_OF_INERTIA;
    public static final DCMotor ARM_MOTOR_CONFIG;
    public static final double ARM_STOW_CONSTANT;
    public static final double ARM_INTAKE_CONSTANT;
    public static final double ARM_AMP_CONSTANT;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_KRAKEN_X60:
      case ROBOT_KRAKEN_X60_PRO:
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
        break;
    }
  }


}
