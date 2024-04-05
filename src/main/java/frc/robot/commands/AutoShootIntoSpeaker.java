// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootIntoSpeaker extends SequentialCommandGroup {
  /** Creates a new AutoShootIntoSpeaker. */
  public AutoShootIntoSpeaker(AdvancedArmSubsystem arm, AdvancedShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    BooleanSupplier armAtSpeakerPosition = () -> arm.getPosition() == Constants.ArmConstants.speakerPosition;

    addCommands(
        new ResetArmPosition(arm),
        new ParallelCommandGroup(
            new ArmToPosition(arm, Constants.ArmConstants.speakerPosition, true),
            new SequentialCommandGroup(
                new WaitUntilCommand(armAtSpeakerPosition),
                shooter.runShooter(Optional.of(5)))));
  }
}
