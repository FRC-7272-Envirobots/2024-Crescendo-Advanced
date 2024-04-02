// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagPID extends Command {
  private static final int TAG = 1;
  private PhotonCamera photonCamera;
  private DriveSubsystem drivetrainSubsystem;
  private PIDController pid;
  private SimpleWidget yawWidget;
  private SimpleWidget areaWidget;
  private SimpleWidget xSpeedWidget;
  private SimpleWidget ySpeedWidget;
  private PIDController pidY;
  private PIDController pidX;
  private boolean doX;
  private boolean doY;
  private double targetTagArea;
  private double targetTagYaw;

  /** Creates a new AprilTagPID. */
  public AprilTagPID(PhotonCamera photonCamera, DriveSubsystem drivetrainSubsystem, int tagId, boolean doX, boolean doY,
      double targetTagArea, double targetTagYaw) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.pidY = new PIDController(.01, 0, .1);
    this.pidX = new PIDController(.01, 0, .1);
    this.doX = doX;
    this.doY = doY;
    this.targetTagArea = targetTagArea;
    this.targetTagYaw = targetTagYaw;
    yawWidget = Shuffleboard.getTab("AptrilTagPID").add("yaw", 0.0);
    areaWidget = Shuffleboard.getTab("AptrilTagPID").add("area", 0.0);
    xSpeedWidget = Shuffleboard.getTab("AptrilTagPID").add("xspeed", 0.0);
    ySpeedWidget = Shuffleboard.getTab("AptrilTagPID").add("yspeed", 0.0);

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidY.reset();
    pidX.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG)
          // .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 &&
          // t.getPoseAmbiguity() != -1)
          .findFirst();

      if (targetOpt.isPresent()) {

        double xSpeed = 0;
        if (doX) {
          // area is the size of the april tag in view
          double currArea = targetOpt.get().getArea();
          areaWidget.getEntry().setDouble(currArea);
          xSpeed = pidX.calculate(currArea, -targetTagArea);
        }

        double ySpeed = 0;
        if (doY) {
          // yaw is the angle offset from target
          double currYaw = targetOpt.get().getYaw();
          yawWidget.getEntry().setDouble(currYaw);
          ySpeed = pid.calculate(currYaw, -targetTagYaw);
        }

        drivetrainSubsystem.drive(xSpeed, ySpeed, 0, true, true);
        xSpeedWidget.getEntry().setDouble(xSpeed);
        ySpeedWidget.getEntry().setDouble(ySpeed);

      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // var photonRes = photonCamera.getLatestResult();
    // var targetOpt = photonRes.getTargets().stream()
    // .filter(t -> t.getFiducialId() == TAG)
    // .findFirst();
    // if (targetOpt.isPresent()) {
    // return false;
    // }
    // return true;
  }
}
