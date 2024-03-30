// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AdvancedArmSubsystem;

public class ArmTo90Deg extends Command {
  /** Creates a new ArmTo90Deg. */
  AdvancedArmSubsystem arm;
  public ArmTo90Deg(AdvancedArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveArm(.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.arm.getEncoderReading() >= 60; //TODO get this;
  }
}
