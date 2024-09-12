package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX motor;
  private final StatusSignal<Double> tempCelsius;
  private final StatusSignal<Double> velocityMPerSec;
  private final StatusSignal<Double> positionMeters;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> appliedVolts;

  private final TalonFXConfiguration motorConfig;

  private final VoltageOut voltageControl;
  private final NeutralOut neutralControl;

  public ClimberIOTalonFX() {
    motor = new TalonFX(ClimberConstants.CLIMBER_CAN_ID);

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.MAX_HEIGHT;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.MIN_HEIGHT;
    motor.getConfigurator().apply(motorConfig);

    tempCelsius = motor.getDeviceTemp();
    velocityMPerSec = motor.getVelocity();
    positionMeters = motor.getPosition();
    currentAmps = motor.getSupplyCurrent();
    appliedVolts = motor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, tempCelsius, velocityMPerSec, positionMeters, currentAmps, appliedVolts);
    motor.optimizeBusUtilization();
    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.positionMeters =
        Math.PI
            * 2
            * ClimberConstants.DRUM_RADIUS
            * positionMeters.getValueAsDouble()
            / ClimberConstants.GEAR_RATIO;
    inputs.velocityMPerSec =
        Math.PI
            * 2
            * ClimberConstants.DRUM_RADIUS
            * velocityMPerSec.getValueAsDouble()
            / ClimberConstants.GEAR_RATIO;
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(MathUtil.clamp(volts, -12.0, 12.0)));
  }

  @Override
  public void stop() {
    setVoltage(0);
    motor.setControl(neutralControl);
  }
}
