// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notused;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagPIDUpBack extends Command {
  private static final int TAG = 1;
  private PhotonCamera photonCamera;
  private DriveSubsystem drivetrainSubsystem;
  // private SimpleWidget yawWidget;
  // private SimpleWidget areaWidget;
  // private SimpleWidget xSpeedWidget;
  // private SimpleWidget ySpeedWidget;
  private PIDController pidX;

  /** Creates a new AprilTagPID. */
  public AprilTagPIDUpBack(PhotonCamera photonCamera, DriveSubsystem drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.pidX = new PIDController(.01, 0, .1);
    addRequirements(drivetrainSubsystem);

    // yawWidget = Shuffleboard.getTab("AptrilTagPIDUpBack").add("yaw", 0.0);
    // areaWidget = Shuffleboard.getTab("AptrilTagPIDUpBack").add("area", 0.0);
    // xSpeedWidget = Shuffleboard.getTab("AptrilTagPIDUpBack").add("xspeed", 0.0);
    // ySpeedWidget = Shuffleboard.getTab("AptrilTagPIDUpBack").add("yspeed", 0.0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double area = .7;
    double yaw = 0;
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG)
          // .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 &&
          // t.getPoseAmbiguity() != -1)
          .findFirst();

      if (targetOpt.isPresent()) {
        // yaw is the angle offset from target
        yaw = targetOpt.get().getYaw();

        // area is the size of the april tag in view
        area = targetOpt.get().getArea();
        //yawWidget.getEntry().setDouble(yaw);
        //areaWidget.getEntry().setDouble(area);

      }
    }

    double xSpeed = pidX.calculate(area, .7);

    drivetrainSubsystem.drive(-xSpeed, 0, 0, true, true);
   // xSpeedWidget.getEntry().setDouble(xSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  //   var photonRes = photonCamera.getLatestResult();
  //   var targetOpt = photonRes.getTargets().stream()
  //       .filter(t -> t.getFiducialId() == TAG)
  //       .findFirst();
  //   if (targetOpt.isPresent()) {
  //     return false;
  //   }
  //   return true;
  }
}
