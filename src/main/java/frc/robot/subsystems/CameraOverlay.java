// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraOverlay extends SubsystemBase {
  Thread m_visionThread;

  // camera dimensions
  private int width = 640;
  private int height = 480;
  // rectangle dimensions
  private double pct_width = .4;
  private double pct_height = .1;
  double midX = width / 2;
  double midY = height / 2;
  double startX = midX - width * pct_width;
  double startY = midY - height * pct_height;
  double endX = midX + width * pct_width;
  double endY = midY + height * pct_height;

  // polygon dimensions
  // top left
  double p1x = midX - width * pct_width;
  double p1y = midY - height * pct_height;

  // top right
  double p2x = midX - width * pct_width;
  double p2y = midY + height * pct_height;

  // bottom left
  double p3x = midX + width * pct_width;
  double p3y = midY + height * pct_height;

  // bottom right
  double p4x = midX + width * pct_width;
  double p4y = midY - height * pct_height;

  /** Creates a new CameraOverlay. */
  public CameraOverlay() {
    m_visionThread = new Thread(
        () -> {

          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture(0);
          // Set the resolution
          camera.setResolution(width, height);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", width, height);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            // Put a rectangle on the image
            Imgproc.rectangle(
                mat, new Point(startX, startY), new Point(endX, endY), new Scalar(0, 255, 0), 1);

            Point p1 = new Point(p1x, p1y);
            Point p2 = new Point(p2x, p2y);
            Point p3 = new Point(p3x, p3y);
            Point p4 = new Point(p4x, p4y);

            double angle1 = 30;

            List<MatOfPoint> list1 = new ArrayList<MatOfPoint>();
            list1.add(new MatOfPoint(
                rotatePoint(p1, angle1, midX, midY),
                rotatePoint(p2, angle1, midX, midY),
                rotatePoint(p3, angle1, midX, midY),
                rotatePoint(p4, angle1, midX, midY)));

            Imgproc.polylines(
                mat, list1, true, new Scalar(0, 255, 255), 1);

            double angle2 = -30;
            List<MatOfPoint> list2 = new ArrayList<MatOfPoint>();
            list2.add(new MatOfPoint(
                rotatePoint(p1, angle2, midX, midY),
                rotatePoint(p2, angle2, midX, midY),
                rotatePoint(p3, angle2, midX, midY),
                rotatePoint(p4, angle2, midX, midY)));

            Imgproc.polylines(
                mat, list2, true, new Scalar(0, 255, 255), 1);

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  public static Point rotatePoint(Point point, double angle, double midx, double midy) {
    double oldX = point.x;
    double oldY = point.y;

    double oldCx = oldX - midx; // move middle to (0,0)
    double oldCy = oldY - midy; // move middle to (0,0)

    double angleRadians = Math.toRadians(angle);

    double angleSin = Math.sin(angleRadians);
    double angleCos = Math.cos(angleRadians);

    double newCx = oldCx * angleCos - oldCy * angleSin;
    double newCy = oldCx * angleSin + oldCy * angleCos;

    double newX = newCx + midx; // move back
    double newY = newCy + midy; // move back

    return new Point((int) newX, (int) newY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
