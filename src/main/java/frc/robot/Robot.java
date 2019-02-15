/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 180;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    // Reset gyro
    vars.gyro.calibrate();
    vars.gyro.reset();

    // Ultrasonic
    vars.leftUltra.setAutomaticMode(true);
    vars.rightUltra.setAutomaticMode(true);

    // Pneumatic
    //vars.compressor.setClosedLoopControl(true);

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    camera.setExposureManual(-1);

    visionThread = new VisionThread(camera, new TapePipeline(), pipeline -> {
      if (pipeline.findContoursOutput().size() == 2) {
        Rect tape1 = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
        Rect tape2 = Imgproc.boundingRect(pipeline.findContoursOutput().get(1));
        synchronized(imgLock) {
          // Alignment phase 1
          int centerX = IMG_WIDTH / 2;
          double areaAvg = ((tape2.area() + tape1.area()) / 2);
          driveSpeed = (2900 - areaAvg) * 0.00007;

          int tape2Distance = ((tape2.x + (tape2.width / 2)) - centerX) * -1;
          int tape1Distance = (tape1.x + (tape1.width / 2)) - centerX;
          sideSpeed = (tape1Distance - tape2Distance) * 0.0015;
        }
        /*synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
        }*/
      } else {
        synchronized (imgLock) {
          // Stop trying if nothing found
          sideSpeed = 0.0;
          driveSpeed = 0.0;
          rotateSpeed = 0.0;
          //centerX = IMG_WIDTH / 2;
        }
      }
    });
    visionThread.start();
  }

  @Override 
  public void robotPeriodic() {
    int leftUltraReading = (int) vars.leftUltra.getRangeMM();
    int rightUltraReading = (int) vars.rightUltra.getRangeMM();
    rotateSpeed = (leftUltraReading - rightUltraReading) * 0.003;
    //System.out.println("Rotate: " + rotateSpeed + "\t/\tForward:" + driveSpeed);
    //System.out.println(/*vars.leftLine.getValue() + */"\t\t/\t\tLeft Ultra: " + leftUltraReading + "\t\tRight Ultra: " + rightUltraReading);
  }

  @Override
  public void autonomousInit() {
    // Read the value of the 'max speed' widget from the dashboard
    vars.drive.setMaxOutput(1.0);
  }

  @Override
  public void autonomousPeriodic() {
    /*double centerX;
    synchronized (imgLock) {
      centerX = this.centerX;
    }
    double turn = centerX - (IMG_WIDTH / 2);
    turn *= 0.0010;
    System.out.print(turn + " / ");
    if(Math.abs(turn) > 0.02) {
      turn += Math.signum(turn) * 0.5;
    } else {
      turn = 0;
    }
    System.out.println(turn);
    vars.drive.driveCartesian(0.0, 0.0, turn);*/
  }

  @Override
  public void teleopPeriodic() {
      // Drive robot
      System.out.println(vars.joystick.getRawAxis(1) * 0.5);
      vars.lift.set(vars.joystick.getRawAxis(1) * 0.5);
      if(vars.lifting == true) {
        /*vars.liftError = vars.targetLiftHeight - vars.frontUltra.getRangeMM();
        vars.liftErrorSum += vars.liftError;
        double targetLiftValue = (vars.liftError * vars.liftP) + (vars.liftErrorSum * vars.liftI);
        vars.lift.set(targetLiftValue);*/
      } else {
      if(vars.driveControl.getRawButton(1)) {
        /*if(Math.abs(sideSpeed) > 0.1) {
          sideSpeed += Math.signum(sideSpeed) * 0.1;
        }
        if(Math.abs(driveSpeed) > 0.1) {
          driveSpeed += Math.signum(driveSpeed) * 0.3;
        }*/
        vars.drive.driveCartesian(sideSpeed, driveSpeed, rotateSpeed);
      } else {
        if(vars.driveControl.getRawButton(2)) {
          vars.claw.set(DoubleSolenoid.Value.kForward);
          System.out.println("Forward");
        } else {
          vars.claw.set(DoubleSolenoid.Value.kReverse);
          System.out.println("Reverse");
        }
        double ySpeed = func.controlCurve(vars.driveControl.getRawAxis(vars.yAxis));
        double xSpeed = func.controlCurve(vars.driveControl.getRawAxis(vars.xAxis)) * -1;
        double zRotation = func.controlCurve(vars.driveControl.getRawAxis(vars.zAxis));
        vars.drive.driveCartesian(ySpeed, xSpeed, zRotation);
        //vars.drive.arcadeDrive(ySpeed, zRotation);
      }
    }
  }
}