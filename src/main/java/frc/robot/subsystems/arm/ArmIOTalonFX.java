package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX motor;
  private final CANcoder cancoder;

  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> velocityRotationsPerSecond;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> temperatureCelcius;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;

  private final StatusSignal<Double> absolutePosition;

  // private final MotionMagicVoltage positionControl;
  private final VoltageOut voltageControl;
  private final NeutralOut neutralControl;

  private final ProfiledPIDController rioPositionControl;
  private ArmFeedforward rioFeedforward;

  private final TalonFXConfiguration motorConfig;
  private final CANcoderConfiguration cancoderConfig;

  private Rotation2d positionGoal;

  private boolean hasResetPosition;

  public ArmIOTalonFX() {
    motor = new TalonFX(ArmConstants.ARM_CAN_ID);
    cancoder = new CANcoder(ArmConstants.CANCODER_CAN_ID);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(ArmConstants.ARM_MAX_VELOCITY.get());
    motorConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(ArmConstants.ARM_MAX_ACCELERATION.get());
    motorConfig.Slot0.kP = ArmConstants.ARM_KP.get();
    motorConfig.Slot0.kD = ArmConstants.ARM_KD.get();
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.Slot0.kS = ArmConstants.ARM_KS.get();
    motorConfig.Slot0.kV = ArmConstants.ARM_KV.get();
    motorConfig.Slot0.kG = ArmConstants.ARM_KG.get();
    motor.getConfigurator().apply(motorConfig);

    cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    positionRotations = motor.getPosition();
    velocityRotationsPerSecond = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();
    temperatureCelcius = motor.getDeviceTemp();
    positionSetpointRotations = motor.getClosedLoopReference();
    positionErrorRotations = motor.getClosedLoopError();

    absolutePosition = cancoder.getAbsolutePosition();

    // positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
    voltageControl = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

    rioPositionControl =
        new ProfiledPIDController(
            ArmConstants.ARM_KP.get(),
            0.0,
            ArmConstants.ARM_KD.get(),
            new Constraints(
                ArmConstants.ARM_MAX_VELOCITY.get(), ArmConstants.ARM_MAX_ACCELERATION.get()));
    rioFeedforward =
        new ArmFeedforward(
            ArmConstants.ARM_KS.get(), ArmConstants.ARM_KG.get(), ArmConstants.ARM_KV.get());
    positionGoal = new Rotation2d();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        currentAmps,
        temperatureCelcius,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePosition);

    motor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    hasResetPosition = false;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        currentAmps,
        temperatureCelcius,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePosition);
    positionSetpointRotations.refresh();
    positionErrorRotations.refresh();

    inputs.armPosition =
        Rotation2d.fromRotations(
            positionRotations.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO);
    inputs.armVelocityRadPerSec =
        Units.rotationsToRadians(
            velocityRotationsPerSecond.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO);
    inputs.armAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.armCurrentAmps = currentAmps.getValueAsDouble();
    inputs.armTemperatureCelsius = temperatureCelcius.getValueAsDouble();

    inputs.armAbsolutePosition =
        Rotation2d.fromRotations(absolutePosition.getValueAsDouble())
            .minus(ArmConstants.ARM_ABSOLUTE_ENCODER_OFFSET);

    // inputs.positionSetpoint =
    //     Rotation2d.fromRotations(
    //         positionSetpointRotations.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO);
    // inputs.positionError =
    //     Rotation2d.fromRotations(
    //         positionErrorRotations.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO);

    inputs.positionSetpoint = Rotation2d.fromRadians(rioPositionControl.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(rioPositionControl.getPositionError());
    inputs.positionGoal = positionGoal;
  }

  @Override
  public void stop() {
    motor.setControl(neutralControl);
  }

  @Override
  public void setArmVoltage(double volts) {
    if (!hasResetPosition) {
      hasResetPosition =
          motor
              .setPosition(
                  (absolutePosition.getValueAsDouble()
                          - ArmConstants.ARM_ABSOLUTE_ENCODER_OFFSET.getRotations())
                      * ArmConstants.ARM_GEAR_RATIO
                      * -1.0)
              .isOK();
    }
    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setArmPosition(Rotation2d currentPosition, Rotation2d setpointPosition) {
    if (!hasResetPosition) {
      hasResetPosition =
          motor
              .setPosition(
                  (absolutePosition.getValueAsDouble()
                          - ArmConstants.ARM_ABSOLUTE_ENCODER_OFFSET.getRotations())
                      * ArmConstants.ARM_GEAR_RATIO
                      * -1.0)
              .isOK();
    }
    positionGoal = setpointPosition;
    // motor.setControl(
    //     positionControl.withPosition(position.getRotations() * ArmConstants.ARM_GEAR_RATIO));
    motor.setControl(
        voltageControl.withOutput(
            rioPositionControl.calculate(
                    currentPosition.getRadians(), setpointPosition.getRadians())
                + rioFeedforward.calculate(
                    setpointPosition.getRadians(), rioPositionControl.getSetpoint().velocity)));
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    motorConfig.Slot0.kP = kp;
    motorConfig.Slot0.kI = ki;
    motorConfig.Slot0.kD = kd;
    motor.getConfigurator().apply(motorConfig);
    rioPositionControl.setPID(kp, ki, kd);
  }

  @Override
  public void setFeedforward(double ks, double kg, double kv) {
    motorConfig.Slot0.kS = ks;
    motorConfig.Slot0.kV = kv;
    motorConfig.Slot0.kG = kg;
    motor.getConfigurator().apply(motorConfig);
    rioFeedforward = new ArmFeedforward(ks, kg, kv);
  }

  @Override
  public void setProfile(double maxVelocity, double maxAcceleration) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    motor.getConfigurator().apply(motorConfig);
    rioPositionControl.setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(
            positionGoal.getRotations()
                - positionRotations.getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO)
        <= Units.degreesToRotations(ArmConstants.GOAL_TOLERANCE.get());
  }
}
