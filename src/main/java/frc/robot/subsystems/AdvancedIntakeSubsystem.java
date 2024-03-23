package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedIntakeSubsystem extends SubsystemBase {
    
    private final TalonFX intake_motor = new TalonFX(13, "");
    private final TalonFX shooter_top_motor = new TalonFX(14, "");
    private final TalonFX shooter_bot_motor = new TalonFX(15, "");

    public AdvancedIntakeSubsystem(){
        super();
        intake_motor.setInverted(true);
        shooter_top_motor.setInverted(false);
        shooter_bot_motor.setInverted(false);
    }

    public Command runIntakeUntilCaptured() {
        return Commands.startEnd(
            ()-> intake_motor.set(.40),
            () -> intake_motor.set(0),
            this
        );
    }

    public Command runShooter() {
        return Commands.startEnd(
            () -> {
                double speed = 0.2;
                // shooter_top_motor.set(speed);
                shooter_bot_motor.set(speed);
                shooter_top_motor.set(speed);
            }, 
            () -> {
                // shooter_top_motor.set(0);
                shooter_bot_motor.set(0);
                shooter_top_motor.set(0);
            },
            this);
    }
}
