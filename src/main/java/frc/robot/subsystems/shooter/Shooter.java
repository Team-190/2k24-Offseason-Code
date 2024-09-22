package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

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

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1.0)),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Shooter/sysIDState", state.toString())),
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

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          io.setTopPID(pid[0], 0.0, pid[1]);
          io.setBottomPID(pid[0], 0.0, pid[1]);
        },
        ShooterConstants.KP,
        ShooterConstants.KD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        ff -> {
          io.setTopFeedForward(ff[0], ff[1], ff[2]);
          io.setBottomFeedForward(ff[0], ff[1], ff[2]);
        },
        ShooterConstants.KS,
        ShooterConstants.KV,
        ShooterConstants.KA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        profile -> {
          io.setTopProfile(profile[0]);
          io.setBottomProfile(profile[0]);
        },
        ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
  }

  public Command setSpeakerVelocity() {

    return Commands.runOnce(
        () -> {
          velocitySetPointRadiansPerSecond = RobotState.getControlData().speakerShotSpeed();
          isClosedLoop = true;
        });
  }

  public Command setFeedVelocity() {

    return Commands.runOnce(
        () -> {
          velocitySetPointRadiansPerSecond = RobotState.getControlData().feedShotSpeed();
          isClosedLoop = true;
        });
  }

  /**
   * Initializes the amp velocity.
   *
   * @return a command to initialize amp velocity.
   */
  public Command setAmpVelocity() {

    return Commands.runOnce(
        () -> {
          velocitySetPointRadiansPerSecond = ShooterConstants.AMP_SPEED.get();
          isClosedLoop = true;
        });
  }

  @AutoLogOutput(key = "Shooter/at setpoint")
  public boolean atSetPoint() {
    return io.atSetPoint();
  }

  /**
   * Runs quasistatic and dynamic tests with the motors moving both forwards and backwards to
   * calculate the feedForward gains.
   *
   * @return the feedFoward gains calculated by the tests
   */
  public Command runCharacterization() {

    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(5),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(5),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(5),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
