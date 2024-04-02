// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.awt.Color;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lightstrip;

public class LightstripEnvirobots extends Command {
  Lightstrip lightstrip;
  Timer timer;

  /** Creates a new LightstripEnvirobots. */
  public LightstripEnvirobots(Lightstrip lightstrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lightstrip = lightstrip;
    addRequirements(lightstrip);
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(timer.get());
    if((Math.round(timer.get()) % 2) == 0){
      lightstrip.setColor(Color.BLUE);
    } else {
        lightstrip.setColor(Color.GREEN);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
