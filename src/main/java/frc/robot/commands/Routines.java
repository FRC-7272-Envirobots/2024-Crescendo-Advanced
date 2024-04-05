// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.awt.Color;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedIntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.NoteIntakeSensor;

/** Add your docs here. */
public class Routines {
    private AdvancedArmSubsystem arm;
    private AdvancedIntakeSubsystem shooter;
    private AdvancedIntakeSubsystem intake;
    private Lightstrip lightstrip;
    private NoteIntakeSensor intakeSensor;

    public Routines(AdvancedArmSubsystem arm, AdvancedIntakeSubsystem shooter, AdvancedIntakeSubsystem intake,
            Lightstrip lightstrip, NoteIntakeSensor intakeSensor) {
        this.arm = arm;
        this.shooter = shooter;
        this.intake = intake;
        this.lightstrip = lightstrip;
        this.intakeSensor = intakeSensor;
    }

    public Command intakeRoutine() {
        return intake.runIntake()
                .alongWith(lightstrip.setColorCommand(Color.RED))
                .until(intakeSensor::isNoteCaptured)
                .andThen(lightstrip.flashColor(Color.WHITE, 0.1, 5.0));
    }
}
