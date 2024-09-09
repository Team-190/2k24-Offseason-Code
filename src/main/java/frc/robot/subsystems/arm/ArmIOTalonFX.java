package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armMotor;

  private final StatusSignal<Double> armPositionRotations;
  private final StatusSignal<Double> armVelocityRotPerSec;
  private final StatusSignal<Double> armAppliedVolts;
  private final StatusSignal<Double> armCurrentAmps;
  private final StatusSignal<Double> armTemperatureCelsius;

  private final TalonFXConfiguration motorConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(ArmConstants.ARM_CAN_ID);

    motorConfig = new TalonFXConfiguration();

    armMotor.getConfigurator().apply(motorConfig);

    armPositionRotations = armMotor.getPosition();
    armVelocityRotPerSec = armMotor.getVelocity();
    armAppliedVolts = armMotor.getMotorVoltage();
    armCurrentAmps = armMotor.getSupplyCurrent();
    armTemperatureCelsius = armMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armPositionRotations,
        armVelocityRotPerSec,
        armAppliedVolts,
        armCurrentAmps,
        armTemperatureCelsius);

    armMotor.optimizeBusUtilization();
    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPosition = Rotation2d.fromRotations(armPositionRotations.getValueAsDouble());
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armVelocityRotPerSec.getValueAsDouble());
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armCurrentAmps = armCurrentAmps.getValueAsDouble();
    inputs.armTemperatureCelsius = armTemperatureCelsius.getValueAsDouble();
  }
}