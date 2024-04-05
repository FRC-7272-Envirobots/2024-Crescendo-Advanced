// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedShooterSubsystem;
import frc.robot.subsystems.Lightstrip;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemCheckRoutine extends SequentialCommandGroup {
  /** Creates a new SystemCheckRoutine. */
  public SystemCheckRoutine(AdvancedArmSubsystem arm, AdvancedShooterSubsystem shooter, Routines routines, Lightstrip lightstrip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetArmPosition(arm),
      new WaitCommand(1),
      new ArmToPosition(arm, Constants.ArmConstants.speakerPosition,false),
      new WaitCommand(1),
      new ArmToPosition(arm, Constants.ArmConstants.ampPosition,false),
      new WaitCommand(1),
      new ArmToPosition(arm, Constants.ArmConstants.intakePosition,false),
      new WaitCommand(1),
      routines.intakeRoutine(),
      shooter.runShooter(.6, Optional.of(5)),
      new StartEndCommand(
        () -> {lightstrip.lightSidesTest();},
        () -> {}
      )
      //new WaitCommand(1)
    );
  }
}
