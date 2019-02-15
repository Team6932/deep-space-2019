/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends TimedRobot {
  private InstanceStorage vars = InstanceStorage.getInstance();
  private CustomFunctions func = CustomFunctions.getInstance();
  private NetworkTableEntry m_maxSpeed;

  private VisionThread visionThread;
  private double driveSpeed = 0.0;
  private double sideSpeed = 0.0;
  private double rotateSpeed = 0.0;

  private double targetLiftHeight = 0;
  private double liftError = 0;
  private double liftErrorSum = 0;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 180;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    // Reset gyro
    vars.gyro.calibrate();
    vars.gyro.reset();

    // Let ultrasonic sensors poll automatically
    vars.leftUltra.setAutomaticMode(true);
    vars.rightUltra.setAutomaticMode(true);

    // Enable the compressor
    // vars.compressor.setClosedLoopControl(true);

    // Initialize camera
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    camera.setExposureManual(-1);

    // Start the vision processing thread for reflective tape detection
    visionThread = new VisionThread(camera, new TapePipeline(), pipeline -> {
      // Get 2 biggest objects found and put them in firstTape / secondTape
      // { index, tape area }
      int[] firstTape = { -1, -1 };
      int[] secondTape = { -1, -1 };
      for (int i = 0; i < pipeline.findContoursOutput().size(); i++) {
        double currentTapeArea = Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).area();
        if (currentTapeArea > firstTape[1]) {
          secondTape = firstTape;
          firstTape = new int[] { i, (int) currentTapeArea };
        }
      }

      // If second tape isn't -1, there have been at least 2 matches found
      if (secondTape[0] != -1) {
        // Grab tape from the index in the arrays
        Rect tape1 = Imgproc.boundingRect(pipeline.findContoursOutput().get(firstTape[0]));
        Rect tape2 = Imgproc.boundingRect(pipeline.findContoursOutput().get(secondTape[0]));

        // Run only in sync with imgLock
        synchronized (imgLock) {
          // Get approximate distance and use it to drive forward
          // Replaced by Ultrasonic drive
          /*
           * double areaAvg = ((tape2.area() + tape1.area()) / 2); driveSpeed = (2900 -
           * areaAvg) * 0.00007;
           */

          // Look at tape and line up horizontally
          int centerX = IMG_WIDTH / 2;
          int tape2Distance = ((tape2.x + (tape2.width / 2)) - centerX) * -1;
          int tape1Distance = (tape1.x + (tape1.width / 2)) - centerX;
          sideSpeed = (tape1Distance - tape2Distance) * 0.0015; // TODO: Fine tune this value
        }
      } else {
        synchronized (imgLock) {
          // Do not move if nothing found
          sideSpeed = 0.0;
          driveSpeed = 0.0;
        }
      }
    });
    visionThread.start();
  }

  @Override
  public void robotPeriodic() {
    int leftUltraReading = (int) vars.leftUltra.getRangeMM();
    int rightUltraReading = (int) vars.rightUltra.getRangeMM();
    int avgReading = (leftUltraReading + rightUltraReading) / 2;

    if (avgReading < 500) {
      // Rotate based on the difference between ultrasonic readings
      rotateSpeed = (leftUltraReading - rightUltraReading) * 0.003; // TODO: Fine tune this value

      // Drive forward based on ultrasonic reading averages
      driveSpeed = avgReading * 0.003; // TODO: Fine tune this value

      // System.out.println("Rotate: " + rotateSpeed + "\t/\tForward:" + driveSpeed);
      // System.out.println(/*vars.leftLine.getValue() + */"\t\t/\t\tLeft Ultra: " +
      // leftUltraReading + "\t\tRight Ultra: " + rightUltraReading);
    } else {
      // Do not rotate if at least not 500mm close
      rotateSpeed = 0;
    }
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    // Drive robot
    System.out.println(vars.joystick.getRawAxis(1) * 0.5);
    vars.lift.set(vars.joystick.getRawAxis(1) * 0.5);

    if (vars.lifting == true) {
      // Basic P-I controller
      liftError = targetLiftHeight - vars.frontUltra.getRangeMM();
      liftErrorSum += liftError;
      double targetLiftValue = (liftError * vars.liftP) + (liftErrorSum * vars.liftI);
      vars.lift.set(targetLiftValue);
    } else {
      // Reset values for P-I controller if we're not lifting
      liftError = 0;
      liftErrorSum = 0;

      // Check if "X" is pressed on controller
      if (vars.driveControl.getRawButton(1)) {
        // Drive with variables set by visionThread + ultrasonic measuring
        vars.drive.driveCartesian(sideSpeed, driveSpeed, rotateSpeed);
      } else {
        // Claw solenoid control
        if (vars.driveControl.getRawButton(2)) {
          vars.claw.set(DoubleSolenoid.Value.kForward);
        } else {
          vars.claw.set(DoubleSolenoid.Value.kReverse);
        }

        // Drive with controller
        double ySpeed = func.controlCurve(vars.driveControl.getRawAxis(vars.yAxis));
        double xSpeed = func.controlCurve(vars.driveControl.getRawAxis(vars.xAxis)) * -1;
        double zRotation = func.controlCurve(vars.driveControl.getRawAxis(vars.zAxis));
        vars.drive.driveCartesian(ySpeed, xSpeed, zRotation);
      }
    }
  }
}