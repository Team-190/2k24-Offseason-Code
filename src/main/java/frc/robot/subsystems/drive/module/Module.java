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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIOInputsAutoLogged inputs;

  private SwerveModulePosition[] odometryPositions;
  private Rotation2d angleSetpoint; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute
  private int cycleCount;

  private final Alert unitializedAlert;
  private final Alert outOfSyncAlert;

  private final ModuleIO io;
  private final int index;

  public Module(ModuleIO io, int index) {
    inputs = new ModuleIOInputsAutoLogged();

    odometryPositions = new SwerveModulePosition[] {};
    angleSetpoint = null;
    speedSetpoint = null;
    turnRelativeOffset = null;
    cycleCount = 0;

    this.io = io;
    this.index = index;

    setBrakeMode(true);

    String moduleName =
        switch (index) {
          case 0 -> "FL";
          case 1 -> "FR";
          case 2 -> "BL";
          case 3 -> "BR";
          default -> "?";
        };
    unitializedAlert =
        new Alert(moduleName + " module turn angle has not been initialized.", AlertType.ERROR);
    outOfSyncAlert =
        new Alert(moduleName + " module turn angle out of sync with CANcoder.", AlertType.ERROR);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Adjust models based on tunable numbers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setDrivePID(pid[0], 0.0, pid[1]),
        ModuleConstants.DRIVE_KP,
        ModuleConstants.DRIVE_KD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setTurnPID(pid[0], 0.0, pid[1]),
        ModuleConstants.TURN_KP,
        ModuleConstants.TURN_KD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        ff -> io.setDriveFeedforward(ff[0], ff[1], 0.0),
        ModuleConstants.DRIVE_KS,
        ModuleConstants.DRIVE_KV);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    cycleCount += 1;
    if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }
    if (cycleCount >= 50) {
      unitializedAlert.set(turnRelativeOffset == null);
    }

    // Alert if out of sync
    if (turnRelativeOffset != null
        && (getAngle().minus(inputs.turnAbsolutePosition).getRadians())
            > ModuleConstants.OUT_OF_SYNC_THRESHOLD) {
      outOfSyncAlert.set(true);
    }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnPositionSetpoint(getAngle(), angleSetpoint);

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        // When the error is 90 degrees, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = speedSetpoint * inputs.turnPositionError.getCos();

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / ModuleConstants.WHEEL_RADIUS.get();
        io.setDriveVelocitySetpoint(inputs.driveVelocityRadPerSec, velocityRadPerSec);
      }
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i].getRadians() * ModuleConstants.WHEEL_RADIUS.get();
      Rotation2d angle =
          inputs.odometryTurnPositions[i].plus(
              turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    Logger.recordOutput("Drive/Module " + index + " Turn Setpoint", angleSetpoint);
    Logger.recordOutput("Drive/Module " + index + " Current Angle", getAngle());
    Logger.recordOutput("Drive/Position " + index, inputs.drivePosition.getRadians());
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPosition.plus(turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePosition.getRadians() * ModuleConstants.WHEEL_RADIUS.get();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * ModuleConstants.WHEEL_RADIUS.get();
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
