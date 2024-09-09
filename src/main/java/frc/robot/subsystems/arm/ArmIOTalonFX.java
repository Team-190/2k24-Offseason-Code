package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.IntakeConstants;

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

  }
}