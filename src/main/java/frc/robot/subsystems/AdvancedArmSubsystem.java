package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedArmSubsystem extends SubsystemBase {
    private final TalonFX left_motor = new TalonFX(12);
    private final TalonFX right_motor = new TalonFX(11);

    private static double move_up_pct_power = 0.4;
    private static double move_down_pct_power = -0.4;
    private static double hold_position_pct_power = 0;

    private DutyCycleEncoder armEncoder;

    PositionVoltage positionControl;
    public AdvancedArmSubsystem() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        left_motor.setInverted(false);
        right_motor.setInverted(true);

        Follower follower = new Follower(left_motor.getDeviceID(), true);
        right_motor.setControl(follower);
        right_motor.getConfigurator().apply(config);
        left_motor.setNeutralMode(NeutralModeValue.Brake);
        right_motor.setNeutralMode(NeutralModeValue.Brake);

        //TODO: Change this to 8. It causes conflicts until the Robot.java encoder is removed.
        armEncoder = new DutyCycleEncoder(7);

        positionControl = new PositionVoltage(0.0);
    }

    public Command moveArmUpCommand() {
        return Commands.startEnd(
                () -> {
                    left_motor.set(move_up_pct_power);
                    right_motor.set(move_up_pct_power);
                },
                () -> {
                    left_motor.set(hold_position_pct_power);
                    //right_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command moveArmDownCommand() {
        return Commands.startEnd(
                () -> {
                    left_motor.set(move_down_pct_power);
                    //right_motor.set(move_down_pct_power);
                },
                () -> {
                    left_motor.set(hold_position_pct_power);
                    //right_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command setBrakeModeCommand() {
        return this.runOnce(() -> {
            left_motor.setNeutralMode(NeutralModeValue.Brake);
            //right_motor.setNeutralMode(NeutralModeValue.Brake);
        });
    }

    public Command setCoastModeCommand() {
        return this.runOnce(() -> {
            left_motor.setNeutralMode(NeutralModeValue.Coast);
            //right_motor.setNeutralMode(NeutralModeValue.Coast);
        });
    }

    public double getEncoderReading() {
        return armEncoder.getPositionOffset();
    }

    public void resetEncoder() {
        armEncoder.reset();
    }

    public void moveArm(double pct_power) {
        left_motor.set(pct_power);
        //right_motor.set(pct_power);
    } 

    public void setPosition(double position) {
        left_motor.setControl(positionControl.withPosition(5.0));
    }
}
