// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.ArmToPositionControl;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedIntakeSubsystem;
import frc.robot.subsystems.AdvancedShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteIntakeSensor;
import frc.robot.subsystems.Lightstrip;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Robot.java components
    private TalonFX left_arm_motor;
    private TalonFX right_arm_motor;

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final NoteIntakeSensor m_lightSensor = new NoteIntakeSensor();
    private final AdvancedShooterSubsystem m_shooter = new AdvancedShooterSubsystem();
    private final AdvancedIntakeSubsystem m_intake = new AdvancedIntakeSubsystem();
    private final AdvancedArmSubsystem m_arm;
    private final Lightstrip lightstrip = new Lightstrip();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    Joystick m_arcadeBox = new Joystick(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(TalonFX left_arm_motor, TalonFX right_arm_motor) {
        this.left_arm_motor = left_arm_motor;
        this.right_arm_motor = right_arm_motor;
        this.m_arm = new AdvancedArmSubsystem(left_arm_motor, right_arm_motor);

        // Configure the button bindings
        configureButtonBindings();

        // Set up camera server
        UsbCamera frontCamera = CameraServer.startAutomaticCapture("front", 0);
        frontCamera.setResolution(640, 480);
        UsbCamera rearCamera = CameraServer.startAutomaticCapture("rear", 1);
        rearCamera.setResolution(640, 480);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(

                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
     * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // new JoystickButton(m_driverController, XboxController.Button.kRB.value)
        // .whileTrue(new RunCommand(
        // () -> m_robotDrive.setX(),
        // m_robotDrive));

        // A button = Run Intake
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
                .whileTrue(
                        m_intake.runIntake()
                                .until(m_lightSensor::isNoteCaptured)
                                .andThen(lightstrip.setIntakeSuccessColor()));

        // X button = Run Shooter
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(Commands.parallel(
                        m_shooter.runShooter(),
                        Commands.sequence(
                                Commands.waitSeconds(.2),
                                m_intake.runIntake()))
                        .finallyDo(() -> lightstrip.setShootCompletedColor()));

        // // Y button = Arm in COAST mode - probably not what you want
        // new JoystickButton(m_driverController, XboxController.Button.kY.value)
        // .onTrue(m_arm.setCoastModeCommand());

        // // B button = Arm in BRAKE mode - this is the default
        // new JoystickButton(m_driverController, XboxController.Button.kB.value)
        // .onTrue(m_arm.setBrakeModeCommand());

        // R1 / RB button - Arm moves up
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                .whileTrue(m_arm.moveArmUpCommand());

        // L1 / LB arrow key - Arm moves down
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                .whileTrue(m_arm.moveArmDownCommand());

        // TODO Test this CAREFULLY. Need to measure encoder values at 90deg first
        // new JoystickButton(m_driverController,
        // XboxController.Axis.kRightTrigger.value)
        // .whileTrue(new ArmTo90Deg(m_arm));

        new JoystickButton(m_arcadeBox, 1)
                .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        new JoystickButton(m_arcadeBox, 2)
                .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        new JoystickButton(m_arcadeBox, 3)
                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kForward));

        new JoystickButton(m_arcadeBox, 4)
                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Shooting angle
        new JoystickButton(m_arcadeBox, 5)
                .whileTrue(new ArmToPositionControl(m_arm, 20, true));
        new JoystickButton(m_arcadeBox,6)
                .whileTrue(new ArmToPosition(m_arm, 20, true));

        // Resting angle
        new JoystickButton(m_arcadeBox, 3)
                .whileTrue(new ArmToPositionControl(m_arm, 82, true));
        new JoystickButton(m_arcadeBox,4)
                .whileTrue(new ArmToPosition(m_arm, 82, true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}
