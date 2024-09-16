package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
 
public class Shooter extends SubsystemBase{

  private final ShooterIOInputsAutoLogged inputs;
  private final ShooterIO io;
  private double velocitySetPointRadiansPerSecond;
  private boolean isClosedLoop;
private final SysIdRoutine characterizationRoutine;
  
  public Shooter(ShooterIO io) {

    inputs = new ShooterIOInputsAutoLogged();
    this.io = io;
    velocitySetPointRadiansPerSecond = 0.0;
    isClosedLoop = true;
    characterizationRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.2).per(Seconds.of(1.0)),
            Volts.of(3.5),
            Seconds.of(10),
            (state) -> Logger.recordOutput("Shooter.sysIDState", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
}
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (isClosedLoop) {

      io.setTopVelocitySetPoint(velocitySetPointRadiansPerSecond);
      io.setBottomVelocitySetPoint(-velocitySetPointRadiansPerSecond);

    }
  }
  
  public Command setSpeakerVelocity() {

    return Commands.runOnce(() -> {

      velocitySetPointRadiansPerSecond = RobotState.getControlData().speakerShotSpeed();
      isClosedLoop = true;

    });

  }
  
  public Command setFeedVelocity() {
    
    return Commands.runOnce( () -> {

      velocitySetPointRadiansPerSecond = RobotState.getControlData().feedShotSpeed();
      isClosedLoop = true;

    });

  }

  public Command setAmpVelocity() {

    return Commands.runOnce(() -> {

      velocitySetPointRadiansPerSecond = ShooterConstants.AMP_SPEED.get();
      isClosedLoop = true;

    });

  }
    
  public boolean atSetPoint() {

    return io.atSetPoint();

  }

  public Command runCharacterization() {

    return Commands.sequence(Commands.runOnce( () -> isClosedLoop = false), 
      characterizationRoutine.quasistatic(Direction.kForward),
      Commands.waitSeconds(10),
      characterizationRoutine.quasistatic(Direction.kReverse),
      Commands.waitSeconds(10),
      characterizationRoutine.dynamic(Direction.kForward),
      Commands.waitSeconds(10),
        characterizationRoutine.dynamic(Direction.kReverse));

  }

  }

