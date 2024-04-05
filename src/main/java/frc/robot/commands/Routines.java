// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.awt.Color;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedIntakeSubsystem;
import frc.robot.subsystems.AdvancedShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.NoteIntakeSensor;

/** Add your docs here. */
public class Routines {
    private AdvancedArmSubsystem arm;
    private AdvancedShooterSubsystem shooter;
    private AdvancedIntakeSubsystem intake;
    private Lightstrip lightstrip;
    private NoteIntakeSensor intakeSensor;
    private DriveSubsystem drive;

    public Routines(AdvancedArmSubsystem arm, AdvancedShooterSubsystem shooter, AdvancedIntakeSubsystem intake,
            Lightstrip lightstrip, NoteIntakeSensor intakeSensor, DriveSubsystem drive) {
        this.arm = arm;
        this.shooter = shooter;
        this.intake = intake;
        this.lightstrip = lightstrip;
        this.intakeSensor = intakeSensor;
        this.drive = drive;
    }

    public Command intakeRoutine() {
        return intake.runIntake()
                .alongWith(lightstrip.setColorCommand(Color.RED))
                .until(intakeSensor::isNoteCaptured)
                .andThen(lightstrip.flashColor(Color.WHITE, 0.1, 5.0));
    }

    public Command shootRoutine(double speed) {
        return Commands.parallel(
                shooter.runShooter(speed, Optional.empty()),
                Commands.sequence(
                                Commands.waitSeconds(.2),
                                intake.runIntake(),
                                new StartEndCommand(
                                                () -> {
                                                        lightstrip.setShootCompletedColor();
                                                },
                                                () -> {
                                                }),
                                new WaitCommand(5)));
    }

    public Command resetHeading() {
       return Commands.run(() -> {this.drive.resetGyroAngle();}, drive);
    }
}
