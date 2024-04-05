// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AdvancedArmSubsystem;

public class ResetArmPosition extends Command {
  private AdvancedArmSubsystem arm;

  /** Creates a new ResetArmPosition. */
  public ResetArmPosition(AdvancedArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveArm(AdvancedArmSubsystem.move_down_pct_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    arm.moveArm(0);
    arm.resetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(arm.reverseLimitSwitched());
    return arm.reverseLimitSwitched();
  }
}
