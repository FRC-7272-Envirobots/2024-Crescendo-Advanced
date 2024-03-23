package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedArmSubsystem extends SubsystemBase {
    private final TalonFX left_motor = new TalonFX(12, "");
    private final TalonFX right_motor = new TalonFX(11, "");

    private static double move_up_pct_power = 0.4;
    private static double move_down_pct_power = -0.4;
    private static double hold_position_pct_power = 0;

    public AdvancedArmSubsystem() {
        super();
        left_motor.setInverted(false);
        right_motor.setInverted(true);

        left_motor.setNeutralMode(NeutralModeValue.Brake);
        right_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command moveArmUpCommand() {
        return Commands.startEnd(
                () -> {
                    left_motor.set(move_up_pct_power);
                    right_motor.set(move_up_pct_power);
                },
                () -> {
                    left_motor.set(hold_position_pct_power);
                    right_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command moveArmDownCommand() {
        return Commands.startEnd(
                () -> {
                    left_motor.set(move_down_pct_power);
                    right_motor.set(move_down_pct_power);
                },
                () -> {
                    left_motor.set(hold_position_pct_power);
                    right_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command setBrakeModeCommand() {
        return this.runOnce(() -> {
            left_motor.setNeutralMode(NeutralModeValue.Brake);
            right_motor.setNeutralMode(NeutralModeValue.Brake);
        });
    }

    public Command setCoastModeCommand() {
        return this.runOnce(() -> {
            left_motor.setNeutralMode(NeutralModeValue.Coast);
            right_motor.setNeutralMode(NeutralModeValue.Coast);
        });
    }

}
