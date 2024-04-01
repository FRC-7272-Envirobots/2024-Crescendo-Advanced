package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

public class AdvancedArmSubsystem extends SubsystemBase {
    private static double move_up_pct_power = 0.4;
    private static double move_down_pct_power = -0.4;
    private static double hold_position_pct_power = 0;

    private final TalonFX left_arm_motor = new TalonFX(12);
    private final TalonFX right_arm_motor = new TalonFX(11);

    TalonFXConfiguration config;
    Follower right_arm_follower;

    private DutyCycleEncoder armEncoder;

    public AdvancedArmSubsystem() {
        super();
        setName("arm");

        // Arm motion logic
        config = new TalonFXConfiguration();
        config.Slot0.kP = 1; // value from SysID tuning: 7.3697 but its too fast and scary;
        // "Nobody uses I" apparently
        // config.Slot0.kI = ;
        config.Slot0.kD = 0.056523;
        config.Slot0.kS = 0.20933;
        config.Slot0.kV = 0.10312;
        config.Slot0.kA = 0.00085311;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = 0.15387;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = left_arm_motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
        right_arm_follower = new Follower(left_arm_motor.getDeviceID(), true);
        right_arm_motor.setControl(right_arm_follower);

        armEncoder = new DutyCycleEncoder(8);

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                left_arm_motor.getPosition(),
                left_arm_motor.getVelocity(),
                left_arm_motor.getMotorVoltage());

        /*
         * Optimize out the other signals, since they're not particularly helpful for us
         */
        left_arm_motor.optimizeBusUtilization();
        SignalLogger.start();
    }

    public Command moveArmUpCommand() {
        return Commands.startEnd(
                () -> {
                    left_arm_motor.set(move_up_pct_power);
                },
                () -> {
                    left_arm_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command moveArmDownCommand() {
        return Commands.startEnd(
                () -> {
                    left_arm_motor.set(move_down_pct_power);
                },
                () -> {
                    left_arm_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command setBrakeModeCommand() {
        return this.runOnce(() -> {
            left_arm_motor.setNeutralMode(NeutralModeValue.Brake);
        });
    }

    public Command setCoastModeCommand() {
        return this.runOnce(() -> {
            left_arm_motor.setNeutralMode(NeutralModeValue.Coast);
        });
    }

    public double getEncoderReading() {
        return armEncoder.getPositionOffset();
    }

    public void resetEncoder() {
        armEncoder.reset();
    }

    public void moveArm(double pct_power) {
        left_arm_motor.set(pct_power);
    }

    public void setControl(PositionVoltage control) {
        left_arm_motor.setControl(control);
    }

    public double getPosition() {
        return left_arm_motor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return left_arm_motor.getVelocity().getValueAsDouble();
    }

    private final VoltageOut m_sysidControl = new VoltageOut(0);
    private SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Default ramp rate is acceptable
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                    null, // Default timeout is acceptable
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> left_arm_motor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
                    null,
                    this));

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }
}
