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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagPID;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.AutoShootIntoSpeaker;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.LightstripEnvirobots;
import frc.robot.commands.Routines;
import frc.robot.commands.notused.AprilTagPIDUpBack;
import frc.robot.subsystems.AdvancedArmSubsystem;
import frc.robot.subsystems.AdvancedIntakeSubsystem;
import frc.robot.subsystems.AdvancedShooterSubsystem;
import frc.robot.subsystems.CameraOverlay;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteIntakeSensor;
import frc.robot.subsystems.Lightstrip;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.awt.Color;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final PhotonCamera photonCamera = new PhotonCamera("7272-limelight-1");
        private final DriveSubsystem m_robotDrive = new DriveSubsystem(photonCamera);
        public final NoteIntakeSensor m_lightSensor = new NoteIntakeSensor();
        private final AdvancedShooterSubsystem m_shooter = new AdvancedShooterSubsystem();
        private final AdvancedIntakeSubsystem m_intake = new AdvancedIntakeSubsystem();
        private final AdvancedArmSubsystem m_arm = new AdvancedArmSubsystem();
        private final Lightstrip lightstrip = new Lightstrip();
        private final Routines routines = new Routines(m_arm, m_shooter, m_intake, lightstrip, m_lightSensor, m_robotDrive);
        // private final CameraOverlay cameraOverlay = new CameraOverlay();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        Joystick m_arcadeBox = new Joystick(1);
        Joystick m_sysIDJoystick = new Joystick(2);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                SmartDashboard.putBoolean("AutoDrive", true);
                // Configure the button bindings
                configureButtonBindings();

                lightstrip.setDefaultCommand(new LightstripEnvirobots(lightstrip));

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
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                OIConstants.kDriveDeadband),
                                                                true, false),
                                                m_robotDrive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
         * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
         * {@link XboxController}), and then passing it to a {@link JoystickButton}.
         */
        private void configureButtonBindings() {

                // A button = Run Intake
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(routines.intakeRoutine());

                // X button = Run Shooter for speaker
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(routines.shootRoutine(.6));

                // Y button = Run Shooter for amp
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(routines.shootRoutine(.2));

                // B button = Arm in BRAKE mode - this is the default
                // ChaseTagCommand chaseTagCommand = new ChaseTagCommand(photonCamera,
                //                 m_robotDrive, () -> m_robotDrive.getPose());

                // new JoystickButton(m_driverController, XboxController.Button.kB.value)
                //                 .whileTrue(chaseTagCommand);
                // new JoystickButton(m_driverController, XboxController.Button.kB.value)
                //                 .whileTrue(chaseTagCommand);
                // R1 / RB button - Arm moves up
                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_arm.moveArmUpCommand());

                // L1 / LB arrow key - Arm moves down
                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(m_arm.moveArmDownCommand());

                new JoystickButton(m_arcadeBox, 1)
                                .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

                // // right
                // new POVButton(m_driverController,
                // 90).whileTrue(m_robotDrive.AprilTagSysIdDynamic(Direction.kForward));

                // // left
                // new POVButton(m_driverController,
                // 270).whileTrue(m_robotDrive.AprilTagSysIdDynamic(Direction.kReverse));

                // top
                new POVButton(m_driverController, 0)
                                .whileTrue(new AprilTagPID(photonCamera, m_robotDrive, 1, true, false, 0, 0));

                // bottom
                new POVButton(m_driverController, 180)
                                .whileTrue(new AprilTagPID(photonCamera, m_robotDrive, 1, false, true, 0, 0));

                // Archade Box Top Buttons
                // Turbo/Macro/Home = Nope
                // Share = Button 7
                // Options = Button 8
                // L3/SL = Nope... Shorted out? Probably 9
                // R3/SR = Button 10

                // Archade Box Buttons
                // L1/LB = Button 5
                // L2/LT = Axis 2
                // X = Button 3
                // A = Button 1
                // Y = Button 4
                // B = Button 2
                // R1/RB = Button 6
                // R2/RT = Axis 3

                EventLoop m_loop = new EventLoop();

                // System Identification Routines
                new JoystickButton(m_sysIDJoystick, 7)
                                .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                new JoystickButton(m_sysIDJoystick, 8)
                                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kForward));

                // If only the button worked.
                new JoystickButton(m_sysIDJoystick, 9)
                                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                new JoystickButton(m_sysIDJoystick, 10)
                                .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));



                // Intake angle
                // Tap (L1/LB) = Button 5
                new JoystickButton(m_arcadeBox, 5)
                                .onTrue(new ArmToPosition(m_arm, Constants.ArmConstants.intakePosition, true));
                // Hold (L2/LT) = Axis 2
                // m_arcadeBox.axisGreaterThan(2, 0.5, m_loop)
                //                 .ifHigh(() -> new ArmToPosition(m_arm,  Constants.ArmConstants.intakePosition, true));

                // Shooting angle
                // Tap (X) = Button 3
                new JoystickButton(m_arcadeBox, 1)
                                .onTrue(new ArmToPosition(m_arm,  Constants.ArmConstants.speakerPosition, true));
                // Hold (A) = Button 1
                // new JoystickButton(m_arcadeBox, 1)
                //                 .whileTrue(new ArmToPosition(m_arm, Constants.ArmConstants.speakerPosition, true));

                // // Resting angle
                // double restingPosition = 0.0;
                // // Tap (Y) = Button 4
                new JoystickButton(m_arcadeBox, 4)
                                .onTrue(new ArmToPosition(m_arm, Constants.ArmConstants.ampPosition, true));
                // // Hold (B) = Button 2
                new JoystickButton(m_arcadeBox, 3)
                                .whileTrue(routines.resetHeading());

                // Tap (R1/RB) = Button 6
                new JoystickButton(m_arcadeBox, 6)
                                .onTrue(new ArmToPosition(m_arm, Constants.ArmConstants.restingPosition, true));

                // // Hold (R2/RT) = Axis 3
                // m_arcadeBox.axisGreaterThan(3, 0.5, m_loop)
                //                 .ifHigh(() -> new ArmToPosition(m_arm, Constants.ArmConstants.ampPosition, true));
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

                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                SendableChooser<Command> routine = new SendableChooser<Command>();
                routine.addOption("SwerveTrajectory",
                                new SwerveControllerCommand(
                                                exampleTrajectory,
                                                m_robotDrive::getPose, // Functional interface to feed supplier
                                                DriveConstants.kDriveKinematics,

                                                // Position controllers
                                                new PIDController(AutoConstants.kPXController, 0, 0),
                                                new PIDController(AutoConstants.kPYController, 0, 0),
                                                thetaController,
                                                m_robotDrive::setModuleStates,
                                                m_robotDrive).andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)));

                routine.addOption("AutoShootOneNote", new AutoShootIntoSpeaker(m_arm, m_shooter, routines, m_robotDrive));
                routine.setDefaultOption("AutoShootOneNote", new AutoShootIntoSpeaker(m_arm, m_shooter, routines, m_robotDrive));
                return routine.getSelected();
        }
}
