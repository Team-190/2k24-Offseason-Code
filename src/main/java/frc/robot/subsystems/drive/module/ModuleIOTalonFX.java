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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.drive.drive.PhoenixOdometryThread;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveTemp;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final StatusSignal<Double> turnTemp;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert cancoderDisconnectedAlert;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(ModuleConstants.frontLeftConfig.drive(), DriveConstants.CANIVORE);
        turnTalon = new TalonFX(ModuleConstants.frontLeftConfig.turn(), DriveConstants.CANIVORE);
        cancoder = new CANcoder(ModuleConstants.frontLeftConfig.encoder(), DriveConstants.CANIVORE);
        absoluteEncoderOffset = ModuleConstants.frontLeftConfig.absoluteEncoderOffset();
        break;
      case 1:
        driveTalon = new TalonFX(ModuleConstants.frontRightConfig.drive(), DriveConstants.CANIVORE);
        turnTalon = new TalonFX(ModuleConstants.frontRightConfig.turn(), DriveConstants.CANIVORE);
        cancoder =
            new CANcoder(ModuleConstants.frontRightConfig.encoder(), DriveConstants.CANIVORE);
        absoluteEncoderOffset = ModuleConstants.frontRightConfig.absoluteEncoderOffset();
        break;
      case 2:
        driveTalon = new TalonFX(ModuleConstants.rearLeftConfig.drive(), DriveConstants.CANIVORE);
        turnTalon = new TalonFX(ModuleConstants.rearLeftConfig.turn(), DriveConstants.CANIVORE);
        cancoder = new CANcoder(ModuleConstants.rearLeftConfig.encoder(), DriveConstants.CANIVORE);
        absoluteEncoderOffset = ModuleConstants.rearLeftConfig.absoluteEncoderOffset();
        break;
      case 3:
        driveTalon = new TalonFX(ModuleConstants.rearRightConfig.drive(), DriveConstants.CANIVORE);
        turnTalon = new TalonFX(ModuleConstants.rearRightConfig.turn(), DriveConstants.CANIVORE);
        cancoder = new CANcoder(ModuleConstants.rearRightConfig.encoder(), DriveConstants.CANIVORE);
        absoluteEncoderOffset = ModuleConstants.rearRightConfig.absoluteEncoderOffset();
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.Audio.AllowMusicDurDisable = true;
    driveConfig.Audio.BeepOnBoot = false;
    driveConfig.Audio.BeepOnConfig = false;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.TURN_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.Audio.AllowMusicDurDisable = true;
    turnConfig.Audio.BeepOnBoot = false;
    turnConfig.Audio.BeepOnConfig = false;

    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();
    turnTemp = turnTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();

    String moduleName =
        switch (index) {
          case 0 -> "FL";
          case 1 -> "FR";
          case 2 -> "BL";
          case 3 -> "BR";
          default -> "?";
        };
    driveDisconnectedAlert =
        new Alert(
            moduleName + " module drive Talon is disconnected, check CAN bus.", AlertType.ERROR);
    turnDisconnectedAlert =
        new Alert(
            moduleName + " module turn Talon is disconnected, check CAN bus.", AlertType.ERROR);
    cancoderDisconnectedAlert =
        new Alert(moduleName + " module CANcoder is disconnected, check CAN bus.", AlertType.ERROR);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    boolean driveConnected =
        BaseStatusSignal.refreshAll(driveVelocity, driveAppliedVolts, driveCurrent, driveTemp)
            .isOK();
    boolean turnConnected =
        BaseStatusSignal.refreshAll(
                turnPosition, turnVelocity, turnAppliedVolts, turnCurrent, turnTemp)
            .isOK();
    boolean cancoderConnected = BaseStatusSignal.refreshAll(turnAbsolutePosition).isOK();
    driveDisconnectedAlert.set(!driveConnected);
    turnDisconnectedAlert.set(!turnConnected);
    cancoderDisconnectedAlert.set(!cancoderConnected);

    inputs.drivePosition =
        Rotation2d.fromRotations(
            drivePosition.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / ModuleConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
    inputs.driveTempCelcius = new double[] {driveTemp.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / ModuleConstants.TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
    inputs.turnTempCelcius = new double[] {turnTemp.getValueAsDouble()};

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
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
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
}
