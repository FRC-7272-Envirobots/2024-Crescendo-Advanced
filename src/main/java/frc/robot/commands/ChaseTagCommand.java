package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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

  private static final int TAG_TO_CHASE = 4;
  private static final Transform3d TAG_TO_GOAL = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final DriveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;
  private SimpleWidget targetFound;
  private SimpleWidget hasTargets;
  private SimpleWidget xSpeedWidget;
  private SimpleWidget ySpeedWidget;
  private SimpleWidget oSpeedWidget;

  private SimpleWidget xTargetPoseWidget;
  private SimpleWidget yTargetPoseWidget;
  private SimpleWidget oTargetPoseWidget;
  private SimpleWidget xGoalPoseWidget;
  private SimpleWidget yGoalPoseWidget;
  private SimpleWidget rGoalPoseWidget;
  private SimpleWidget xDrivePoseWidget;
  private SimpleWidget yDrivePoseWidget;
  private SimpleWidget zDrivePoseWidget;
  private SimpleWidget vxChassisWidget;
  private SimpleWidget vyChassisWidget;

  private SimpleWidget xControllerGoalWidget;
  private SimpleWidget yControllerGoalWidget;
  private SimpleWidget oControllerGoalWidget;

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

    // Construct PhotonPoseEstimator: This is for field relative driving. Not used YET.
    Transform3d robotToCam = new Transform3d( 
      new Translation3d(0.5, 0.0, 0.5),  //TODO: Cam mounted facing forward, ? meters forward of center, ? meters right of center, ? meters up from center.
      new Rotation3d(0,0,0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, robotToCam);

    // ShuffleBoard
    this.targetFound = Shuffleboard.getTab("chase").add("target-found", false);
    this.hasTargets = Shuffleboard.getTab("chase").add("has-targets", false);

    this.xSpeedWidget = Shuffleboard.getTab("chase").add("speed-x", 0.0);
    this.ySpeedWidget = Shuffleboard.getTab("chase").add("speed-y", 0.0);
    this.oSpeedWidget = Shuffleboard.getTab("chase").add("speed-o", 0.0);

    this.xDrivePoseWidget = Shuffleboard.getTab("chase").add("drive-pose-x", 0.0);
    this.yDrivePoseWidget = Shuffleboard.getTab("chase").add("drive-pose-y", 0.0);
    this.zDrivePoseWidget = Shuffleboard.getTab("chase").add("drive-pose-z", 0.0);

    this.xTargetPoseWidget = Shuffleboard.getTab("chase").add("target-pose-x", 0.0);
    this.yTargetPoseWidget = Shuffleboard.getTab("chase").add("target-pose-y", 0.0);
    this.oTargetPoseWidget = Shuffleboard.getTab("chase").add("target-pose-o", 0.0);

    this.xGoalPoseWidget = Shuffleboard.getTab("chase").add("goal-pose-x", 0.0);
    this.yGoalPoseWidget = Shuffleboard.getTab("chase").add("goal-pose-y", 0.0);
    this.rGoalPoseWidget = Shuffleboard.getTab("chase").add("goal-pose-r", 0.0);

    this.xControllerGoalWidget = Shuffleboard.getTab("chase").add("controller-x-atgoal", false);
    this.yControllerGoalWidget = Shuffleboard.getTab("chase").add("controller-y-atgoal", false);
    this.oControllerGoalWidget = Shuffleboard.getTab("chase").add("controller-o-atgoal", false);

    this.vxChassisWidget = Shuffleboard.getTab("chase").add("chassis-vx", 0.0);
    this.vyChassisWidget = Shuffleboard.getTab("chase").add("chassis-vy", 0.0);

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

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    xDrivePoseWidget.getEntry().setDouble(robotPose.getX());
    yDrivePoseWidget.getEntry().setDouble(robotPose.getY());
    zDrivePoseWidget.getEntry().setDouble(robotPose.getZ());

    var photonRes = photonCamera.getLatestResult();
    hasTargets.getEntry().setBoolean(photonRes.hasTargets());
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          // .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 &&
          // t.getPoseAmbiguity() != -1)
          .findFirst();

      targetFound.getEntry().setBoolean(targetOpt.isPresent());
      if (targetOpt.isPresent()) {

        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        xTargetPoseWidget.getEntry().setDouble(targetPose.getX());
        yTargetPoseWidget.getEntry().setDouble(targetPose.getY());
        oTargetPoseWidget.getEntry().setDouble(targetPose.getZ());

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
        xGoalPoseWidget.getEntry().setDouble(goalPose.getX());
        yGoalPoseWidget.getEntry().setDouble(goalPose.getY());
        rGoalPoseWidget.getEntry().setDouble(goalPose.getRotation().getRadians());

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());

        xControllerGoalWidget.getEntry().setBoolean(xController.atGoal());
        yControllerGoalWidget.getEntry().setBoolean(yController.atGoal());
        oControllerGoalWidget.getEntry().setBoolean(omegaController.atGoal());
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

      xSpeedWidget.getEntry("speed-x").setDouble(xSpeed);
      ySpeedWidget.getEntry("speed-y").setDouble(ySpeed);
      oSpeedWidget.getEntry("speed-o").setDouble(omegaSpeed);

      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
          robotPose2d.getRotation());

      vxChassisWidget.getEntry("chassis-vx").setDouble(chassisSpeeds.vxMetersPerSecond);
      vyChassisWidget.getEntry("chassis-vy").setDouble(chassisSpeeds.vyMetersPerSecond);
      drivetrainSubsystem.driveChassisSpeeds(chassisSpeeds);

    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.driveChassisSpeeds(new ChassisSpeeds());
  }

}