// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lightstrip extends SubsystemBase{

  private static final int PWM_PORT = 9;
  private static final int LENGTH = 300;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private final NoteIntakeSensor noteIntakeSensor; 

  private boolean lastSensorValue;

  /** Creates a new Launcher. */
  public Lightstrip(NoteIntakeSensor noteIntakeSensor) {
    super();
    this.noteIntakeSensor = noteIntakeSensor;

    this.m_led = new AddressableLED(PWM_PORT);

    // Reuse buffer
    // Length is expensive to set, so only set it once, then just update data
    this. m_ledBuffer = new AddressableLEDBuffer(LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    this.setColor(Color.kGreen);
  }

  public Command setIntakeSuccessColor() {
    return runOnce(() -> setColor(Color.kOrangeRed));
  }

  public void setShootCompletedColor() {
    setColor(Color.kGreen);
  }

  private void setColor(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      // m_ledBuffer.setLED(i, color);
      m_ledBuffer.setLED(i, color);
   }
   m_led.setData(m_ledBuffer);
  }

  // @Override
  // public void periodic() {
  //   // if 
  // }
}
