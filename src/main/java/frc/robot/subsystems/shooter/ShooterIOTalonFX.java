package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterIOTalonFX implements ShooterIO {
  
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final StatusSignal<Double> topPositionRotations;
  private final StatusSignal<Double> topVelocityRotPerSec;
  private final StatusSignal<Double> topAppliedVolts;
  private final StatusSignal<Double> topCurrentAmps;
  private final StatusSignal<Double> topTemperatureCelsius;
  private final StatusSignal<Double> topVelocityErrorRotationsPerSec;

  private final StatusSignal<Double> bottomPositionRotations;
  private final StatusSignal<Double> bottomVelocityRotPerSec;
  private final StatusSignal<Double> bottomAppliedVolts;
  private final StatusSignal<Double> bottomCurrentAmps;
  private final StatusSignal<Double> bottomTemperatureCelsius;
  private final StatusSignal<Double> bottomVelocityErrorRotationsPerSec;

  private final TalonFXConfiguration topConfig;
  private final TalonFXConfiguration bottomConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage topProfiledVelocityControl;
  private final MotionMagicVelocityVoltage bottomProfiledVelocityControl;

  private double topSetPointRadiansPerSecond;
  private double bottomSetPointRadiansPerSecond;

  public ShooterIOTalonFX() {

    topMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    topConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    topConfig.Slot0.kP = ShooterConstants.KP.get();
    topConfig.Slot0.kD = ShooterConstants.KD.get();
    topConfig.Slot0.kS = ShooterConstants.KS.get();
    topConfig.Slot0.kV = ShooterConstants.KV.get();
    topConfig.MotionMagic.MotionMagicAcceleration = 
        Units.radiansToRotations(ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());
      
    bottomConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bottomConfig.Slot0.kP = ShooterConstants.KP.get();
    bottomConfig.Slot0.kD = ShooterConstants.KD.get();
    bottomConfig.Slot0.kS = ShooterConstants.KS.get();
    bottomConfig.Slot0.kV = ShooterConstants.KV.get();
    bottomConfig.MotionMagic.MotionMagicAcceleration = 
      Units.radiansToRotations(ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());
      
    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);

    topPositionRotations = topMotor.getPosition();
    topVelocityRotPerSec = topMotor.getVelocity();
    topAppliedVolts = topMotor.getMotorVoltage();
    topCurrentAmps = topMotor.getSupplyCurrent();
    topTemperatureCelsius = topMotor.getDeviceTemp();

    bottomPositionRotations = bottomMotor.getPosition();
    bottomVelocityRotPerSec = bottomMotor.getVelocity();
    bottomAppliedVolts = bottomMotor.getMotorVoltage();
    bottomCurrentAmps = bottomMotor.getSupplyCurrent();
    bottomTemperatureCelsius = bottomMotor.getDeviceTemp();

    topVelocityErrorRotationsPerSec = topMotor.getClosedLoopError();
    bottomVelocityErrorRotationsPerSec = bottomMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
    50.0, 
    topPositionRotations, 
    topVelocityRotPerSec, 
    topAppliedVolts, 
    topCurrentAmps,
    topTemperatureCelsius,
    bottomPositionRotations,
    bottomVelocityRotPerSec,
    bottomAppliedVolts,
    bottomCurrentAmps,
    bottomTemperatureCelsius);
    
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();

    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);
    topProfiledVelocityControl = new MotionMagicVelocityVoltage(0);
    bottomProfiledVelocityControl = new MotionMagicVelocityVoltage(0);

  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRotPerSec.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();
    inputs.topVelocityErrorRadiansPerSec = Units
        .rotationsToRadians(topVelocityErrorRotationsPerSec.getValueAsDouble() / ShooterConstants.TOP_GEAR_RATIO);

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadPerSec = Units
        .rotationsPerMinuteToRadiansPerSecond(bottomVelocityRotPerSec.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();
    inputs.bottomVelocityErrorRadiansPerSec = Units
        .rotationsToRadians(bottomVelocityErrorRotationsPerSec.getValueAsDouble() / ShooterConstants.BOTTOM_GEAR_RATIO);

  }
  
  @Override
  public void setTopVelocitySetPoint(double setPointVelocityRadiansPerSecond) {

    topSetPointRadiansPerSecond = setPointVelocityRadiansPerSecond;
    topMotor.setControl(topProfiledVelocityControl.withVelocity(Units.radiansToRotations(topSetPointRadiansPerSecond)));

  }

  @Override
  public void setBottomVelocitySetPoint(double setPointVelocityRadiansPerSecond) {

    bottomSetPointRadiansPerSecond = setPointVelocityRadiansPerSecond;
    bottomMotor.setControl(
        bottomProfiledVelocityControl.withVelocity(Units.radiansToRotations(bottomSetPointRadiansPerSecond)));

  }
  
  @Override
  public void setTopFeedForward(double kS, double kV, double kA) {

    topConfig.Slot0.kS = kS;
    topConfig.Slot0.kV = kV;
    topConfig.Slot0.kA = kA;
    topMotor.getConfigurator().apply(topConfig, 0.01);

  }
  
  @Override
  public void setBottomFeedForward(double kS, double kV, double kA) {

    topConfig.Slot0.kS = kS;
    topConfig.Slot0.kV = kV;
    topConfig.Slot0.kA = kA;
    topMotor.getConfigurator().apply(topConfig, 0.01);

  }

  @Override
  public boolean atSetPoint() {

    double topSetPointRadiansPerSecondUpperBound = topSetPointRadiansPerSecond +
        (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND.get() / 2);
    double topSetPointRadiansPerSecondLowerBound = topSetPointRadiansPerSecond -
        (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND.get() / 2);
    double bottomSetPointRadiansPerSecondUpperBound = bottomSetPointRadiansPerSecond +
        (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND.get() / 2);
    double bottomSetPointRadiansPerSecondLowerBound = bottomSetPointRadiansPerSecond -
        (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND.get() / 2);

    double currentTopVelocityRadiansPerSecond = Units.rotationsToRadians(topVelocityRotPerSec.getValueAsDouble());
    double currentBottomVelocityRadiansPerSecond = Units.rotationsToRadians(bottomVelocityRotPerSec.getValueAsDouble());

    return currentTopVelocityRadiansPerSecond < topSetPointRadiansPerSecondUpperBound
        && currentTopVelocityRadiansPerSecond > topSetPointRadiansPerSecondLowerBound
        && currentBottomVelocityRadiansPerSecond < bottomSetPointRadiansPerSecondUpperBound
        && currentBottomVelocityRadiansPerSecond > bottomSetPointRadiansPerSecondLowerBound;

  }
  
  @Override
  public void setTopProfile(double maxAccelerationRadiansPerSecondSquared) {

    topConfig.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    topMotor.getConfigurator().apply(topConfig);

  }

  @Override
  public void setBottomProfile(double maxAccelerationRadiansPerSecondSquared) {

    bottomConfig.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    bottomMotor.getConfigurator().apply(bottomConfig);

  }

  @Override
  public void setTopPID(double kP, double kI, double kD) {

    topConfig.Slot0.kP = kP;
    topConfig.Slot0.kI = kI;
    topConfig.Slot0.kD = kD;
    topMotor.getConfigurator().apply(topConfig, 0.001);

  }
  
   @Override
   public void setBottomPID(double kP, double kI, double kD) {

     bottomConfig.Slot0.kP = kP;
     bottomConfig.Slot0.kI = kI;
     bottomConfig.Slot0.kD = kD;
     bottomMotor.getConfigurator().apply(bottomConfig, 0.001);

   }
  
   @Override
   public void setVoltage(double volts) {

    topMotor.setControl(voltageControl.withOutput(volts));
    bottomMotor.setControl(voltageControl.withOutput(volts));

   }

   @Override
   public void stop() {

     topMotor.setControl(neutralControl);
     bottomMotor.setControl(neutralControl);

   }

}
