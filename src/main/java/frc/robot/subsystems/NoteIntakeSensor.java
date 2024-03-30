// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class NoteIntakeSensor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 m_colorSensor;
  private ColorMatch m_colorMatcher;

  private final double colorConfidenceThreshold = 0.84;
  private final double proximityThreshold = 150;

  // This color corresponds to the Orange color of the KNote.
  private final Color kNoteTarget = new Color(0.53,0.37,0.09);

  /** Creates a new LightSensor. */
  public NoteIntakeSensor() {
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(kNoteTarget);
    m_colorMatcher.setConfidenceThreshold(colorConfidenceThreshold);
  }

  /**
   * Returns whether the kNote is visible from the light sensor.
   * @return boolean
   */
  public boolean isNoteCaptured() {
    int proximity = m_colorSensor.getProximity();
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
    boolean result =  match != null && match.color == kNoteTarget && proximity > proximityThreshold;
    return result;
  }

  // @Override
  // public void periodic() {
  //   int proximity = m_colorSensor.getProximity();
  //   Color detectedColor = m_colorSensor.getColor();
  //   ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
  //   SmartDashboard.putNumber("color-proximity", proximity);
  //   SmartDashboard.putString("color-name", detectedColor.toString());
  //   SmartDashboard.putString("color-match-result", match == null ? "" : match.toString());
  //   boolean result =  match != null && match.color == kNoteTarget && proximity > proximityThreshold;
  //   SmartDashboard.putBoolean("color-result", result);
  // }
}
