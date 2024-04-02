// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lightstrip extends SubsystemBase {

  private static final int PWM_PORT = 9;
  private static final int LENGTH = 300;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int start_red = 0;
  private int end_red = 41;
  private int start_green = 41;
  private int end_green = 67;
  private int start_blue = 67;
  private int end_blue = 106;
  private Timer timer = new Timer();

  public void alternate(Color color1, Color color2) {
    timer.restart();

  }

  /** Creates a new Launcher. */
  public Lightstrip() {
    super();
    this.m_led = new AddressableLED(PWM_PORT);

    // Reuse buffer
    // Length is expensive to set, so only set it once, then just update data
    this.m_ledBuffer = new AddressableLEDBuffer(LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    this.setColor(Color.GREEN);
  }

  public Command flashColor(Color color, double seconds, double duration) {
    timer.reset();
    timer.start();
    return run(() -> {
      if (timer.get() % seconds < 0.01) {
        setColor(color);
      } else {
        setColor(Color.BLACK);
      }
    }).withTimeout(duration);
  }


  public Command setIntakeSuccessColor() {
    return run(() -> setColor(Color.WHITE)).withTimeout(5);
  }

  public void setShootCompletedColor() {
    setColor(Color.GREEN);
  }

  public void lightSidesTest() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if (i <= 41) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      } else if (i > 41 && i < 67) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      } else if (i >= 67 && i < 106) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      } else {
        m_ledBuffer.setRGB(i, 20, 20, 20);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public Command setColorCommand(Color color) {
    return run(() -> setColor(color));
  }

  public void setColor(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
    m_led.setData(m_ledBuffer);
  }

}
