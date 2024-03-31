package frc.robot;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ChaseTagCommand extends Command {

  // 32 x 27 chassis outer perimeter
  //
  // Coordinates docs:
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(Units.inchesToMeters(-16), 0,
      Units.inchesToMeters(3), new Rotation3d(0, Units.degreesToRadians(30), Math.toRadians(180)));

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = new Transform3d(
      new Translation3d(1.5, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final DriveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

  public ChaseTagCommand(
      PhotonCamera photonCamera,
      DriveSubsystem drivetrainSubsystem,
      Supplier<Pose2d> poseProvider) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    Shuffleboard.getTab("chase").add("drive-pose", robotPose2d);

    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();

      Shuffleboard.getTab("chase").add("target-found", targetOpt.isPresent());
      if (targetOpt.isPresent()) {

        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        Shuffleboard.getTab("chase").add("cam-to-target", camToTarget);

        var targetPose = cameraPose.transformBy(camToTarget);
        Shuffleboard.getTab("chase").add("target-pose", targetPose);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
        Shuffleboard.getTab("chase").add("goal-pose", goalPose);

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }

    if (lastTarget == null) {
      // No target has been visible
      drivetrainSubsystem.driveChassisSpeeds(new ChassisSpeeds());
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      Shuffleboard.getTab("chase").add("controller-x", xController);
      Shuffleboard.getTab("chase").add("controller-y", yController);
      Shuffleboard.getTab("chase").add("controller-o", omegaController);

      Shuffleboard.getTab("chase").add("speed-x", xSpeed);
      Shuffleboard.getTab("chase").add("speed-y", ySpeed);
      Shuffleboard.getTab("chase").add("speed-o", omegaSpeed);

      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
          robotPose2d.getRotation());
      Shuffleboard.getTab("chase").add("chassis-speeds", chassisSpeeds);
      // drivetrainSubsystem.driveChassisSpeeds(chassisSpeeds);

    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.driveChassisSpeeds(new ChassisSpeeds());
  }

}