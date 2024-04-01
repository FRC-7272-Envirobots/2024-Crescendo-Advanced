package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedShooterSubsystem extends SubsystemBase {

    private final TalonFX shooter_top_motor = new TalonFX(14, "");
    private final TalonFX shooter_bot_motor = new TalonFX(15, "");

    public AdvancedShooterSubsystem() {
        super();
        shooter_top_motor.setInverted(false);
        shooter_bot_motor.setInverted(false);
    }

    public Command runShooter() {
        return Commands.startEnd(
                () -> {
                    double speed = 0.6;
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
