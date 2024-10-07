// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Leds extends SubsystemBase {

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hasNote = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  // Constants
  private static final int length = 32;

  public Leds() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    solid(RobotState.getControlData().hasNote() ? Color.kGreen : Color.kBlack);
    // Update LEDs
    leds.setData(buffer);
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
      }
    }
  }
}