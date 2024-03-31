package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
    private TalonFX left_motor;
    private TalonFX right_motor;

    private DutyCycleEncoder armEncoder;

    // PositionVoltage positionControl;

    public AdvancedArmSubsystem(TalonFX left_motor, TalonFX right_motor) {
        super();
        this.left_motor = left_motor;
        this.right_motor = right_motor;
        armEncoder = new DutyCycleEncoder(8);
        // positionControl = new PositionVoltage(0.0);

        setName("arm");

        // TalonFXConfiguration cfg = new TalonFXConfiguration();
        // left_motor.getConfigurator().apply(cfg);

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                left_motor.getPosition(),
                left_motor.getVelocity(),
                left_motor.getMotorVoltage());

        /*
         * Optimize out the other signals, since they're not particularly helpful for us
         */
        left_motor.optimizeBusUtilization();

        SignalLogger.start();
    }

    public Command moveArmUpCommand() {
        return Commands.startEnd(
                () -> {
                    left_motor.set(move_up_pct_power);
                },
                () -> {
                    left_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command moveArmDownCommand() {
        return Commands.startEnd(
                () -> {
                    left_motor.set(move_down_pct_power);
                },
                () -> {
                    left_motor.set(hold_position_pct_power);
                }, this);
    }

    public Command setBrakeModeCommand() {
        return this.runOnce(() -> {
            left_motor.setNeutralMode(NeutralModeValue.Brake);
        });
    }

    public Command setCoastModeCommand() {
        return this.runOnce(() -> {
            left_motor.setNeutralMode(NeutralModeValue.Coast);
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
    }

    // public void setPosition(double position) {
    // left_motor.setControl(positionControl.withPosition(position).withVelocity(35).withSlot(0));
    // }

    public void setControl(PositionVoltage control) {
        left_motor.setControl(control);
    }

    public double getPosition() {
        return left_motor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return left_motor.getVelocity().getValueAsDouble();
    }
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    // private final MutableMeasure<Voltage> m_appliedVoltage =
    // mutable(Volts.of(0));
    // // Mutable holder for unit-safe linear distance values, persisted to avoid
    // // reallocation.
    // private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // // reallocation.
    // private final MutableMeasure<Velocity<Angle>> m_velocity =
    // mutable(RotationsPerSecond.of(0));

    // // Create a new SysId routine for characterizing the shooter.
    // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    // // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
    // new SysIdRoutine.Config()

    // ),
    // new SysIdRoutine.Mechanism(
    // // Tell SysId how to plumb the driving voltage to the motor(s).
    // (Measure<Voltage> volts) -> {
    // left_motor.setVoltage(volts.in(Volts));
    // },
    // // Tell SysId how to record a frame of data for each motor on the mechanism
    // // being
    // // characterized.
    // log -> {
    // // Record a frame for the shooter motor.
    // SignalLogger.writeString("state", "arm-left");
    // SignalLogger.writeDouble("voltage",
    // left_motor.getMotorVoltage().getValueAsDouble() *
    // RobotController.getBatteryVoltage());
    // SignalLogger.writeDouble("position",
    // left_motor.getPosition().getValueAsDouble());
    // SignalLogger.writeDouble("velocity",
    // left_motor.getRotorVelocity().getValueAsDouble());
    // },
    // // Tell SysId to make generated commands require this subsystem, suffix test
    // // state in
    // // WPILog with this subsystem's name ("shooter")
    // this));

    private final VoltageOut m_sysidControl = new VoltageOut(0);
    private SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Default ramp rate is acceptable
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                    null, // Default timeout is acceptable
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> left_motor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
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
