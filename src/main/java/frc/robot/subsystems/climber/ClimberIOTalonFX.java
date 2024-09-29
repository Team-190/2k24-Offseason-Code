package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX motor;
  private final StatusSignal<Double> tempCelsius;
  private final StatusSignal<Double> velocityRadiansPerSecond;
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> appliedVolts;

  private final TalonFXConfiguration motorConfig;

  private final VoltageOut voltageControl;
  private final NeutralOut neutralControl;

  private boolean hasSetPosition;

  /**
   * Creates the TalonFX motor object and configures it. Sets the variables from the motor and
   * optimizes the can bus utilization.
   */
  public ClimberIOTalonFX() {
    motor = new TalonFX(ClimberConstants.CLIMBER_CAN_ID);

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(motorConfig);

    tempCelsius = motor.getDeviceTemp();
    velocityRadiansPerSecond = motor.getVelocity();
    positionRotations = motor.getPosition();
    currentAmps = motor.getSupplyCurrent();
    appliedVolts = motor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, tempCelsius, velocityRadiansPerSecond, positionRotations, currentAmps, appliedVolts);
    motor.optimizeBusUtilization();
    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0);

    hasSetPosition = false;
  }

  /**
   * Updates inputs in AdvantageKit. Called in @see periodic().
   *
   * @param inputs are the input variables logged in AdvantageKit. These include applied voltage,
   *     current in amps, position in meters, velocity in meters per second, and the motor
   *     temperature in celsius
   */
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        tempCelsius, velocityRadiansPerSecond, positionRotations, currentAmps, appliedVolts);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.position =
        Rotation2d.fromRotations(
            positionRotations.getValueAsDouble() / ClimberConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        velocityRadiansPerSecond.getValueAsDouble() / ClimberConstants.GEAR_RATIO;
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  /**
   * Sets the output voltage to apply to the motor.
   *
   * @param volts volts to apply to the motor as a double
   */
  @Override
  public void setVoltage(double volts) {
    if (!hasSetPosition) {
      hasSetPosition = motor.setPosition(0.0).isOK();
    }
    motor.setControl(voltageControl.withOutput(volts));
  }

  /** Sets the volts to 0 and sets motor Control to neutral. */
  @Override
  public void stop() {
    setVoltage(0);
    motor.setControl(neutralControl);
  }
}
