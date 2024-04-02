// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notused;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AdvancedArmSubsystem;

public class ArmToPositionControl extends Command {
  private AdvancedArmSubsystem arm;
  private double position;
  private boolean continuous;

  // class member variable
  final PositionVoltage m_position = new PositionVoltage(0);
  // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
  final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(35, 160));

  TrapezoidProfile.State m_goal;
  TrapezoidProfile.State m_setpoint;

  /** Creates a new ArmToShooterAngle. */
  public ArmToPositionControl(AdvancedArmSubsystem arm, double position, boolean continuous) {
    this.arm = arm;
    this.position = position;
    this.continuous = continuous;
    m_goal = new TrapezoidProfile.State(position, 35);
    m_setpoint = new TrapezoidProfile.State();
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("exec");
    // periodic, update the profile setpoint for 20 ms loop time
    m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    // apply the setpoint to the control request
    m_position.Position = m_setpoint.position;
    m_position.Velocity = m_setpoint.velocity;

    System.out.println("setpoint position: " + m_setpoint.position);
    System.out.println("setpoint velocity: " + m_setpoint.velocity);

    m_position.Slot = 0;
    arm.setControl(m_position);
    System.out.println("arm curr position: " + arm.getPosition());
    System.out.println("arm curr velocity: " + arm.getVelocity());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
