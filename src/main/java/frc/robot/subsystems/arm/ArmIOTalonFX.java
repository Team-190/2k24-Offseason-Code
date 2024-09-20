package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armMotor;
  private final CANcoder cancoder;

  private final StatusSignal<Double> armPositionRotations;
  private final StatusSignal<Double> armVelocityRotPerSec;
  private final StatusSignal<Double> armAppliedVolts;
  private final StatusSignal<Double> armCurrentAmps;
  private final StatusSignal<Double> armTemperatureCelsius;
  private final StatusSignal<Double> armAbsolutePositionRotations;

  private final MotionMagicVoltage armProfiledPositionControl;

  private final TalonFXConfiguration motorConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(ArmConstants.ARM_CAN_ID);
    cancoder = new CANcoder(ArmConstants.ARM_ENCODER_ID);

    motorConfig = new TalonFXConfiguration();

    armMotor.getConfigurator().apply(motorConfig);
    armMotor.setPosition(cancoder.getPosition().getValueAsDouble());

    armPositionRotations = armMotor.getPosition();
    armVelocityRotPerSec = armMotor.getVelocity();
    armAppliedVolts = armMotor.getMotorVoltage();
    armCurrentAmps = armMotor.getSupplyCurrent();
    armTemperatureCelsius = armMotor.getDeviceTemp();
    armAbsolutePositionRotations = cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armPositionRotations,
        armVelocityRotPerSec,
        armAppliedVolts,
        armCurrentAmps,
        armTemperatureCelsius,
        armAbsolutePositionRotations);

    armMotor.optimizeBusUtilization();
    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);

    armProfiledPositionControl = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPosition = Rotation2d.fromRotations(armPositionRotations.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO);
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armVelocityRotPerSec.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO);
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armCurrentAmps = armCurrentAmps.getValueAsDouble();
    inputs.armTemperatureCelsius = armTemperatureCelsius.getValueAsDouble();

    inputs.armAbsolutePosition =
        Rotation2d.fromRotations(armAbsolutePositionRotations.getValueAsDouble())
            .minus(ArmConstants.ARM_ABSOLUTE_ENCODER_OFFSET);
  }

  @Override
  public void setArmVoltage(double volts) {
    armMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    armMotor.setControl(neutralControl);
  }

  /**
   * Retrieves the current angle of the arm motor.
   *
   * <p>This function retrieves the current angle of the arm motor, measured in rotations, from the
   * absolute position sensor. The absolute position sensor provides an accurate and reliable
   * measurement of the arm motor's angle, even when the motor is not moving.
   *
   * @return The current angle of the arm motor in rotations. The angle is relative to the absolute
   *     encoder offset, which is defined in the {@link ArmConstants} class.
   */
  @Override
  public double getCurrentAngle() {
    return armAbsolutePositionRotations.getValueAsDouble();
  }

  /**
   * Sets the position setpoint for the arm motor using motion magic control.
   *
   * <p>This function sets the position setpoint for the arm motor using motion magic control.
   * Motion magic control allows the arm motor to follow a trajectory defined by a set of setpoints,
   * providing smoother and more efficient motion compared to position or velocity control.
   *
   * <p>The motion magic control algorithm continuously calculates the error between the current
   * position and the setpoint, and applies a control signal to the motor based on the proportional,
   * integral, and derivative gains. Additionally, the algorithm takes into account the maximum
   * velocity and acceleration of the motor to ensure that the motion is smooth and efficient.
   *
   * @param position The desired position setpoint for the arm motor in rotations. This value should
   *     be within the motor's specified range of motion.
   */
  @Override
  public void setArmPosition(double position) {
    armMotor.setControl(armProfiledPositionControl.withPosition(position));
  }

  /**
   * Sets the PID gains for the arm motor.
   *
   * <p>PID control is a widely used technique in control systems to improve the performance of the
   * system. It compensates for the dynamics of the system, such as inertia and friction, by
   * continuously adjusting the control signal based on the error between the setpoint and the
   * current position. This helps to achieve faster response times and better tracking of the
   * setpoints.
   *
   * <p>This function sets the proportional (kp), integral (ki), and derivative (kd) gains for the
   * arm motor. The PID gains are used in conjunction with the feedforward gains to compute the
   * control signal for the motor.
   *
   * @param kp The proportional gain. This gain is multiplied by the error between the setpoint and
   *     the current position to generate a proportional control signal. A higher kp value increases
   *     the sensitivity to the error, but may cause overshoot.
   * @param ki The integral gain. This gain is multiplied by the integral of the error between the
   *     setpoint and the current position to generate an integral control signal. A higher ki value
   *     reduces the steady-state error, but may cause oscillations.
   * @param kd The derivative gain. This gain is multiplied by the derivative of the error between
   *     the setpoint and the current position to generate a derivative control signal. A higher kd
   *     value reduces the settling time, but may cause overshoot.
   */
  @Override
  public void setPID(double kp, double ki, double kd) {
    motorConfig.Slot0.kP = kp;
    motorConfig.Slot0.kI = ki;
    motorConfig.Slot0.kD = kd;
    armMotor.getConfigurator().apply(motorConfig, 0.01);
  }

  /**
   * Sets the feedforward gains for the arm motor.
   *
   * <p>Feedforward control is a technique used in control systems to improve the performance of the
   * system. It compensates for the dynamics of the system, such as inertia and friction, by
   * directly applying control signals based on the desired setpoints. This helps to achieve faster
   * response times and better tracking of the setpoints.
   *
   * <p>This function sets the proportional (kS), integral (kG), and velocity (kV) feedforward gains
   * for the arm motor. The feedforward gains are used in conjunction with the PID gains to compute
   * the control signal for the motor.
   *
   * @param ks The proportional feedforward gain. This gain is multiplied by the error between the
   *     setpoint and the current position to generate a proportional control signal.
   * @param kg The integral feedforward gain. This gain is multiplied by the integral of the error
   *     between the setpoint and the current position to generate an integral control signal.
   * @param kv The velocity feedforward gain. This gain is multiplied by the desired velocity
   *     setpoint to generate a velocity control signal.
   */
  @Override
  public void setFeedforward(double ks, double kg, double kv) {
    motorConfig.Slot0.kS = ks;
    motorConfig.Slot0.kG = kg;
    motorConfig.Slot0.kV = kv;
    armMotor.getConfigurator().apply(motorConfig, 0.01);
  }

  /**
   * Sets the motion profile parameters for the arm motor.
   *
   * <p>This function configures the maximum velocity and acceleration for the arm motor's motion
   * profile. The motion profile allows the arm motor to follow a trajectory defined by a set of
   * setpoints, providing smoother and more efficient motion compared to position or velocity
   * control.
   *
   * @param max_velocity The maximum velocity of the arm motor in rotations per minute. This value
   *     should be within the motor's specified maximum velocity.
   * @param max_acceleration The maximum acceleration of the arm motor in rotations per minute
   *     squared. This value should be within the motor's specified maximum acceleration.
   */
  @Override
  public void setProfile(double max_velocity, double max_acceleration) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = max_velocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = max_acceleration;
    armMotor.getConfigurator().apply(motorConfig, 0.01);
  }
}
