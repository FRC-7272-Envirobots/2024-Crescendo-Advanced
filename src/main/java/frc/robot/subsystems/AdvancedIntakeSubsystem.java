package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedIntakeSubsystem extends SubsystemBase {
    
    private final TalonFX intake_motor = new TalonFX(6, "");
    private final TalonFX shooter_top_motor = new TalonFX(7, "");
    private final TalonFX shooter_bot_motor = new TalonFX(5, "");

    public AdvancedIntakeSubsystem(){
        super();
        shooter_top_motor.setInverted(false);
        shooter_bot_motor.setInverted(true);
    }

    public Command runIntakeUntilCaptured() {
        return Commands.startEnd(
            ()-> intake_motor.set(.1),
            () -> intake_motor.set(0),
            this
        );
    }

    public Command runShooter() {
        return Commands.startEnd(
            () -> {
                double speed = 0.1;
                // shooter_top_motor.set(speed);
                shooter_bot_motor.set(speed);
            }, 
            () -> {
                // shooter_top_motor.set(0);
                shooter_bot_motor.set(0);
            },
            this);
    }
}
