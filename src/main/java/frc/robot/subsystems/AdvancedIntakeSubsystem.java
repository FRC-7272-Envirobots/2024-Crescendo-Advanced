package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedIntakeSubsystem extends SubsystemBase {
    
    private final TalonFX intake_motor = new TalonFX(13, "");

    public AdvancedIntakeSubsystem(){
        super();
        intake_motor.setInverted(true);
    }

    public Command runIntake() {
        return Commands.startEnd(
            ()-> intake_motor.set(.50),
            () -> intake_motor.set(0),
            this
        );
    }
}
