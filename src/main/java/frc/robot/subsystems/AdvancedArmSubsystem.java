package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class AdvancedArmSubsystem extends SubsystemBase {
    public static double move_up_pct_power = 0.4;
    public static double move_down_pct_power = -0.4;
    public static double hold_position_pct_power = 0;
    public double speakerPosition = Constants.ArmConstants.speakerPosition;

    private final TalonFX left_arm_motor = new TalonFX(12);
    private final TalonFX right_arm_motor = new TalonFX(11);

    TalonFXConfiguration config;
    Follower right_arm_follower;

    DigitalInput limitSwitch1;
    DigitalInput limitSwitch2;


    public AdvancedArmSubsystem() {
        super();
        this.speakerPosition = SmartDashboard.getNumber("speakerPosition", Constants.ArmConstants.speakerPosition);
        setName("arm");

        limitSwitch1 = new DigitalInput(7);
        limitSwitch2 = new DigitalInput(8);

        // Arm motion logic
        config = new TalonFXConfiguration();
        config.Slot0.kP = 1; // value from SysID tuning: 7.3697 but its too fast and scary;
        // "Nobody uses I" apparently
        // config.Slot0.kI = ;
        config.Slot0.kD = 0.056523;
        config.Slot0.kS = 0.20933;
        config.Slot0.kV = 0.10312;
        config.Slot0.kA = 0.00085311;

        //Limit Switch
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable=true;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 82.0; //TOD: CHANGE THIS TO CORRECT VALUE

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
        // reset relative position of motor to zero.
        // Robot MUST be in start position when robot is turned on, or press the reset button on roborio.
        left_arm_motor.setPosition(0);
        left_arm_motor.setNeutralMode(NeutralModeValue.Brake);

        right_arm_follower = new Follower(left_arm_motor.getDeviceID(), true);
        right_arm_motor.setNeutralMode(NeutralModeValue.Brake);
        right_arm_motor.setControl(right_arm_follower);

        //armEncoder = new DutyCycleEncoder(8);

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                left_arm_motor.getPosition(),
                left_arm_motor.getVelocity(),
                left_arm_motor.getMotorVoltage());

        /*
         * Optimize out the other signals, since they're not particularly helpful for us
         */
        left_arm_motor.optimizeBusUtilization();
    }

    public Command moveArmUpCommand() {
        return Commands.startEnd(
                () -> {
                    System.out.println("moving up");
                    left_arm_motor.set(move_up_pct_power);
                },
                () -> {
                    System.out.println("moving stopped");
                    left_arm_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command moveArmDownCommand() {
        return Commands.startEnd(
                () -> {
                    // if(!endstopTriggered()) {
                        left_arm_motor.set(move_down_pct_power);
                    // } else {
                    //     left_arm_motor.set(hold_position_pct_power);
                    // }
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

    // public boolean endstopTriggered() {
    //     return limitSwitch1.get() || limitSwitch2.get();
    // }

    // public double getEncoderReading() {
    //     return armEncoder.getPositionOffset();
    // }

    // public void resetEncoder() {
    //     armEncoder.reset();
    // }

    public void moveArm(double pct_power) {
        left_arm_motor.set(pct_power);
    }

    public void setControl(ControlRequest control) {
        left_arm_motor.setControl(control);
    }

    public boolean forwardLimitSwitched() {
       return left_arm_motor.getForwardLimit().getValue().value == ForwardLimitValue.ClosedToGround.value;
    }

    public boolean reverseLimitSwitched() {
        return left_arm_motor.getReverseLimit().getValue().value == ReverseLimitValue.ClosedToGround.value;
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
                SignalLogger.start();

        return m_SysIdRoutine.quasistatic(direction);
        // SignalLogger.stop();
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
                SignalLogger.start();

        return m_SysIdRoutine.dynamic(direction);
        // SignalLogger.stop();
    }

    public void resetPosition() {
        left_arm_motor.setPosition(0);
    }
}
