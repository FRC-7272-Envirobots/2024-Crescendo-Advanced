// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AdvancedArmSubsystem;

public class ArmToPosition extends Command {
  private AdvancedArmSubsystem arm;
  private double position;
  private boolean continuous;

  // class member variable
  final PositionVoltage m_position =  new PositionVoltage(0);
  // final MotionMagicVoltage m_magic = new MotionMagicVoltage(0;)
  // final trapa

  /** Creates a new ArmToShooterAngle. */
  public ArmToPosition(AdvancedArmSubsystem arm, double position, boolean continuous) {
    this.arm = arm;
    this.position = position;
    this.continuous = continuous;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("init position");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("exec");
    m_position.Slot = 0;
    m_position.Position = position;
    //m_position.Velocity = 35;
    arm.setControl(m_position);
    // System.out.println("arm curr position: " + arm.getPosition());
    // System.out.println("arm curr velocity: " + arm.getVelocity());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveArm(AdvancedArmSubsystem.hold_position_pct_power);
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Never end if continuous, use command timeouts or whileTrue on buttons
    if (continuous) {
      return false;
    }
    // End within tolerance of 1 position when not continuous
    return arm.getPosition() <= position + 1 && arm.getPosition() >= position - 1;
  }
}
