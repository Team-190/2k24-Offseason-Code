package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.drive.DriveConstants;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armMotor;
  private final CANcoder cancoder;


  private final StatusSignal<Double> armPositionRotations;
  private final StatusSignal<Double> armVelocityRotPerSec;
  private final StatusSignal<Double> armAppliedVolts;
  private final StatusSignal<Double> armCurrentAmps;
  private final StatusSignal<Double> armTemperatureCelsius;
  private final StatusSignal<Double> armAbsolutePositionRotations;

  private final TalonFXConfiguration motorConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(ArmConstants.ARM_CAN_ID);
    cancoder = new CANcoder(ArmConstants.ARM_ENCODER_ID, DriveConstants.CANIVORE);


    motorConfig = new TalonFXConfiguration();

    armMotor.getConfigurator().apply(motorConfig);

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
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPosition = Rotation2d.fromRotations(armPositionRotations.getValueAsDouble());
    inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(armVelocityRotPerSec.getValueAsDouble());
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

  @Override
  public double getCurrentAngle() {
    return armAbsolutePositionRotations.getValueAsDouble();
  }
}