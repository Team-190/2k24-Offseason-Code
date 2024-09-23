// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.drive.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.module.ModuleConstants.ModuleConfig;
import java.util.Queue;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final StatusSignal<Double> drivePositionRotations;
  private final StatusSignal<Double> driveVelocityRotPerSec;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveTemp;

  private final StatusSignal<Double> turnAbsolutePositionRotations;
  private final StatusSignal<Double> turnPositionRotations;
  private final StatusSignal<Double> turnVelocityRotPerSec;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final StatusSignal<Double> turnTemp;

  private final StatusSignal<Double> driveVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> turnPositionSetpointRotations;

  private final StatusSignal<Double> driveVelocityErrorRotationsPerSecond;
  private final StatusSignal<Double> turnPositionErrorRotations;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;
  private final VelocityVoltage velocityControl;
  // private final PositionVoltage positionControl;

  // private SimpleMotorFeedforward driveFeedforward;
  // private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  public ModuleIOTalonFX(ModuleConfig moduleConfig) {
    driveTalon = new TalonFX(moduleConfig.drive(), DriveConstants.CANIVORE);
    turnTalon = new TalonFX(moduleConfig.turn(), DriveConstants.CANIVORE);
    cancoder = new CANcoder(moduleConfig.encoder(), DriveConstants.CANIVORE);
    absoluteEncoderOffset = moduleConfig.absoluteEncoderOffset();

    driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_KS.get();
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_KV.get();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_KP.get();
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_KD.get();

    driveTalon.getConfigurator().apply(driveConfig, 1);

    turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.TURN_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // turnConfig.Slot0.kP = ModuleConstants.TURN_KP.get();
    // turnConfig.Slot0.kD = ModuleConstants.TURN_KD.get();

    turnTalon.getConfigurator().apply(turnConfig, 1);

    for (int i = 0; i < 4; i++) {
      boolean error = driveTalon.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
      error = error && (turnTalon.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
      if (!error) break;
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.1);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());

    drivePositionRotations = driveTalon.getPosition();
    driveVelocityRotPerSec = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();

    turnAbsolutePositionRotations = cancoder.getAbsolutePosition();
    turnPositionRotations = turnTalon.getPosition();
    turnVelocityRotPerSec = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();
    turnTemp = turnTalon.getDeviceTemp();

    driveVelocitySetpointRotationsPerSecond = driveTalon.getClosedLoopReference();
    turnPositionSetpointRotations = turnTalon.getClosedLoopReference();

    driveVelocityErrorRotationsPerSecond = driveTalon.getClosedLoopError();
    turnPositionErrorRotations = turnTalon.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.ODOMETRY_FREQUENCY,
        drivePositionRotations,
        turnPositionRotations,
        driveVelocityRotPerSec,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePositionRotations,
        turnVelocityRotPerSec,
        turnAppliedVolts,
        turnCurrent,
        turnTemp,
        driveVelocitySetpointRotationsPerSecond,
        turnPositionSetpointRotations,
        driveVelocityErrorRotationsPerSecond,
        turnPositionErrorRotations);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
    voltageControl = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    velocityControl = new VelocityVoltage(0.0).withUpdateFreqHz(0.0);
    // positionControl = new PositionVoltage(0.0).withUpdateFreqHz(0.0);

    // driveFeedforward =
    //     new SimpleMotorFeedforward(ModuleConstants.DRIVE_KS.get(),
    // ModuleConstants.DRIVE_KV.get());
    // driveFeedback =
    //     new PIDController(ModuleConstants.DRIVE_KP.get(), 0, ModuleConstants.DRIVE_KD.get());
    turnFeedback =
        new PIDController(ModuleConstants.TURN_KP.get(), 0, ModuleConstants.TURN_KD.get());
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePositionRotations,
        turnPositionRotations,
        driveVelocityRotPerSec,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePositionRotations,
        turnVelocityRotPerSec,
        turnAppliedVolts,
        turnCurrent,
        turnTemp,
        driveVelocitySetpointRotationsPerSecond,
        turnPositionSetpointRotations,
        driveVelocityErrorRotationsPerSecond,
        turnPositionErrorRotations);

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRotations(value / ModuleConstants.DRIVE_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> Rotation2d.fromRotations(value / ModuleConstants.TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    inputs.drivePosition =
        Rotation2d.fromRotations(
            drivePositionRotations.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(
            driveVelocityRotPerSec.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveTempCelcius = driveTemp.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePositionRotations.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnPositionRotations.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(
            turnVelocityRotPerSec.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnTempCelcius = turnTemp.getValueAsDouble();

    inputs.driveVelocitySetpointRadPerSec =
        Units.rotationsToRadians(
            driveVelocitySetpointRotationsPerSecond.getValueAsDouble()
                / ModuleConstants.DRIVE_GEAR_RATIO);
    // inputs.turnPositionSetpoint =
    //     Rotation2d.fromRotations(
    //         turnPositionSetpointRotations.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);

    inputs.driveVelocityErrorRadPerSec =
        Units.rotationsToRadians(
            driveVelocityErrorRotationsPerSecond.getValueAsDouble()
                / ModuleConstants.DRIVE_GEAR_RATIO);
    // inputs.turnPositionError =
    //     Rotation2d.fromRotations(
    //         turnPositionErrorRotations.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);

    // inputs.driveVelocitySetpointRadPerSec = driveFeedback.getSetpoint();
    inputs.turnPositionSetpoint = Rotation2d.fromRadians(turnFeedback.getSetpoint());

    // inputs.driveVelocityErrorRadPerSec = driveFeedback.getPositionError();
    inputs.turnPositionError = Rotation2d.fromRadians(turnFeedback.getPositionError());
  }

  @Override
  public void setDriveVelocitySetpoint(
      double currentVelocityRadPerSec, double setpointVelocityRadsPerSec) {
    driveTalon.setControl(
        velocityControl.withVelocity(
            Units.radiansToRotations(
                setpointVelocityRadsPerSec * ModuleConstants.DRIVE_GEAR_RATIO)));
    // driveTalon.setControl(
    //     voltageControl.withOutput(
    //         driveFeedback.calculate(currentVelocityRadPerSec, setpointVelocityRadsPerSec)
    //             + driveFeedforward.calculate(setpointVelocityRadsPerSec)));
  }

  @Override
  public void setTurnPositionSetpoint(Rotation2d currentPosition, Rotation2d setpointPosition) {
    // turnTalon.setControl(
    //     positionControl.withPosition(
    //         setpointPosition.getRotations() * ModuleConstants.TURN_GEAR_RATIO));
    turnTalon.setControl(
        voltageControl.withOutput(
            turnFeedback.calculate(currentPosition.getRadians(), setpointPosition.getRadians())));
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveConfig.Slot0, 1);
    // driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    // turnConfig.Slot0.kP = kP;
    // turnConfig.Slot0.kI = kI;
    // turnConfig.Slot0.kD = kD;
    // turnTalon.getConfigurator().apply(turnConfig.Slot0, 1);
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveFeedforward(double kS, double kV, double kA) {
    driveConfig.Slot0.kS = kS;
    driveConfig.Slot0.kV = kV;
    driveConfig.Slot0.kA = kA;
    driveTalon.getConfigurator().apply(driveConfig.Slot0, 1);
    // driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.Clockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    driveTalon.setControl(neutralControl);
    turnTalon.setControl(neutralControl);
  }
}
