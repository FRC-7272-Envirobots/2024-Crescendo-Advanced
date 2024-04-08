// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.awt.List;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootIntoSpeaker extends SequentialCommandGroup {
  /** Creates a new AutoShootIntoSpeaker. */
  public AutoShootIntoSpeaker(AdvancedArmSubsystem arm, AdvancedShooterSubsystem shooter, Routines routines,
      DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // BooleanSupplier armAtSpeakerPosition = () -> arm.getPosition() ==
    // Constants.ArmConstants.speakerPosition;

    ArrayList<Command> cmds = new ArrayList<Command>();

    
    cmds.add(new ResetArmPosition(arm));
    cmds.add(new WaitCommand(1));
    cmds.add(new ArmToPosition(arm, Constants.ArmConstants.speakerPosition + 9, false));
    cmds.add(new WaitCommand(.5));
    // new SequentialCommandGroup(
    // new WaitUntilCommand(armAtSpeakerPosition),
    cmds.add(routines.shootRoutine(.6).withTimeout(2));

    // );

      // IF YOU WANT TO DRIVE BACK IN AUTO CHANGE THIS SO ITS NOT COMMENTED
      // IT WAITS 1 SECONDS THEN DRIVES AT 40% SPEED FOR 1.4 SECONDS, THEN STOPS.
      // IF WE ARE POSITIONED NOT ON THE FRONT OF THE SPEAKER, WE SHOULD INCREASE TO 2 SECONDS TO MAKE SURE WE DRIVE FAR ENOUGH
      // System.out.print("auto started");
      // cmds.add(new WaitCommand(1));
      // cmds.add(new StartEndCommand(() -> {
      //       drive.drive(.4, 0, 0, true, false);
      //     }, () -> {
      //       drive.drive(0, 0, 0, true, false);
      //     }, drive).withTimeout(1.4));
    
    Command[] cmdsarr = new Command[cmds.size()];
    cmds.toArray(cmdsarr);
    addCommands(cmdsarr);
  }
}
