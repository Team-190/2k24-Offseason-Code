package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs;
  private final IntakeIO io;

  private final Timer doubleTimer;

  public Intake(IntakeIO io) {
    inputs = new IntakeIOInputsAutoLogged();
    this.io = io;

    doubleTimer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (hasNoteLocked() && !hasNoteStaged()) {
      io.setTopVoltage(-3.0);
      doubleTimer.stop();
      doubleTimer.reset();
    }

    if (hasNoteLocked() && hasNoteStaged() && doubleTimer.get() <= 3.0) {
      io.setTopVoltage(-1.0);
    }

    if (hasNoteLocked() && hasNoteStaged() && doubleTimer.get() <= 0.0) {
      io.setTopVoltage(0.0);
      doubleTimer.start();
    }

    if (doubleTimer.get() >= 0.0 && !hasNoteStaged() && !hasNoteLocked()) {
      io.setTopVoltage(0.0);
      doubleTimer.stop();
      doubleTimer.reset();
    }

    if (hasNoteLocked() && hasNoteStaged() && doubleTimer.get() >= 3.0) {
      io.setTopVoltage(12.0);
    }

    Logger.recordOutput("Intake/Timer", doubleTimer.get());
  }

  public boolean hasNoteLocked() {
    return inputs.middleSensor || inputs.finalSensor;
  }

  public boolean hasNoteStaged() {
    return inputs.intakeSensor;
  }

  public Command intake() {
    return Commands.sequence(
            Commands.parallel(
                    Commands.runEnd(() -> io.setTopVoltage(-12.0), () -> io.setTopVoltage(0.0)),
                    Commands.runEnd(
                        () -> io.setBottomVoltage(12.0), () -> io.setBottomVoltage(0.0)),
                    Commands.runEnd(
                        () -> io.setAcceleratorVoltage(12.0), () -> io.setAcceleratorVoltage(0.0)))
                .until(() -> inputs.middleSensor),
            Commands.parallel(
                    Commands.runEnd(() -> io.setTopVoltage(-1.5), () -> io.setTopVoltage(0.0)),
                    Commands.runEnd(() -> io.setBottomVoltage(1.5), () -> io.setBottomVoltage(0.0)),
                    Commands.runEnd(
                        () -> io.setAcceleratorVoltage(1.5), () -> io.setAcceleratorVoltage(0.0)))
                .until(() -> inputs.finalSensor))
        .until(() -> inputs.finalSensor);
  }

  public Command eject() {
    return Commands.parallel(
        Commands.runEnd(() -> io.setTopVoltage(12.0), () -> io.setTopVoltage(0.0)),
        Commands.runEnd(() -> io.setBottomVoltage(-12.0), () -> io.setBottomVoltage(0.0)),
        Commands.runEnd(
            () -> io.setAcceleratorVoltage(-12.0), () -> io.setAcceleratorVoltage(0.0)));
  }

  public Command shoot() {
    return Commands.runEnd(
            () -> io.setAcceleratorVoltage(12.0), () -> io.setAcceleratorVoltage(0.0))
        .withTimeout(0.25);
  }
}
