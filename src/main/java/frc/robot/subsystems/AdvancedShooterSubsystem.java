package frc.robot.subsystems;

import java.util.Optional;

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

    public Command runShooter(double speed, Optional<Integer> seconds) {

        Command shoot = Commands.startEnd(
                () -> {
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

        if (!seconds.isEmpty()) {
            return shoot.withTimeout(seconds.get());
        }

        return shoot;
    }
}
