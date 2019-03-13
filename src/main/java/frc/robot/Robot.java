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
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends TimedRobot {
  private InstanceStorage vars = InstanceStorage.getInstance();
  private CustomFunctions func = CustomFunctions.getInstance();

  private VisionThread visionThread;
  private VisionThread groundVisionThread;
  Timer timer = new Timer();
  Timer timer2 = new Timer();
  private double driveSpeed = 0.0;
  private double sideSpeed = 0.0;
  private double rotateSpeed = 0.0;

  private double leftUltraReading = 0;
  private double rightUltraReading = 0;

  private double targetLiftHeight = 0;
  private double liftError = 0;
  private double liftErrorSum = 0;

  private double rotateError = 0;

  private double forwardError = 0;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 180;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    // Reset gyro
    vars.gyro.reset();
    vars.gyro.calibrate();

    vars.drive.setSafetyEnabled(false); // TODO: Do not leave this enabled

    // Let ultrasonic sensors poll automatically
    vars.leftUltra.setAutomaticMode(true);
    vars.rightUltra.setAutomaticMode(true);
    vars.frontUltra.setAutomaticMode(true);

    // Enable the compressor
    //vars.compressor.setClosedLoopControl(true);
  }

  @Override
  public void robotPeriodic() {
    // Ultrasonic drive system
    if (vars.leftUltra.isRangeValid()) {
      leftUltraReading = (int) vars.leftUltra.getRangeMM();
    } else {
      leftUltraReading -= 5;
    }
    if (vars.rightUltra.isRangeValid()) {
      rightUltraReading = (int) vars.rightUltra.getRangeMM();
    } else {
      rightUltraReading -= 5;
    }
    /*
     * if (vars.frontUltra.isRangeValid()) { frontUltraReading = (int)
     * vars.frontUltra.getRangeMM(); }
     */
    // System.out.println(leftUltraReading + " / " + rightUltraReading);
    double avgReading = (leftUltraReading + rightUltraReading) / 2;

    if (avgReading < 700) {
      // Rotate based on the difference between ultrasonic readings
      rotateError = leftUltraReading - rightUltraReading;
      rotateSpeed = (rotateError * vars.rotateP); // TODO: Fine tune this value

      // Drive forward based on ultrasonic reading averages
      forwardError = avgReading - 240;
      //driveSpeed = (forwardError * vars.ultraP); // TODO: Fine tune this value

      // System.out.println("Rotate: " + rotateSpeed + "\t/\tForward:" + driveSpeed);
      // System.out.println(/*vars.leftLine.getValue() + */"\t\t/\t\tLeft Ultra: " +
      // leftUltraReading + "\t\tRight Ultra: " + rightUltraReading);
    } else {
      // Do not rotate or drive if at least not 500mm close
      rotateSpeed = 0;
      //driveSpeed = 0;
    }
    // System.out.println(rotateSpeed + " / " + driveSpeed);
    sideSpeed = (vars.leftLine.getVoltage() - (5 - vars.rightLine.getVoltage())) * 0.25;
    //System.out.println(leftUltraReading + " / " + rightUltraReading + " / " + driveSpeed);
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    /*if (timer.get() < 2.0) {
      vars.lift.set(0.2);
    } else if (timer.get() >= 2.0 && timer.get() < 3.0) {
      vars.claw.set(DoubleSolenoid.Value.kForward);
    } else if (timer.get() >= 3.0 && timer.get() < 4.0) {
      vars.claw.set(DoubleSolenoid.Value.kReverse);
    } else if (timer.get() >= 4.0 && timer.get() < 6.0) {
      vars.lift.set(-0.15);
    } else {
      vars.lift.set(0);
    }TODO: Enable*/
  }

  @Override
  public void teleopInit() {
    timer2.reset();
    timer2.start();
  }

  @Override
  public void teleopPeriodic() {
    // Drive robot
    vars.lift.set(vars.joystick.getRawAxis(1) * 0.5);

    if (vars.lifting == true) {
      // Basic P-I controller
      liftError = targetLiftHeight - vars.frontUltra.getRangeMM();
      double targetLiftValue = (liftError * vars.liftP) + (liftErrorSum * vars.liftI);
      //vars.lift.set(targetLiftValue);
      //vars.drive.driveCartesian(0, 0, 0);
    } else {

      // Check if "X" is pressed on controller
      if (vars.driveControl.getRawButton(1)) {
        // Drive with variables set by visionThread + ultrasonic measuring
        if (Math.abs(sideSpeed) > 0.3) {
          sideSpeed = Math.signum(sideSpeed) * 0.3;
        }
        if (Math.abs(driveSpeed) > 0.25) {
          driveSpeed = Math.signum(driveSpeed) * 0.25;
        }
        if (Math.abs(rotateSpeed) > 0.3) {
          rotateSpeed = Math.signum(rotateSpeed) * 0.3;
        }
        //vars.drive.driveCartesian(sideSpeed, driveSpeed, rotateSpeed);
        System.out.println(sideSpeed + " / " + driveSpeed + " / " + rotateSpeed);
      } else {
        // Claw solenoid control
        /*if (vars.driveControl.getRawButton(2)) {
          vars.claw.set(DoubleSolenoid.Value.kForward);
        } else {
          vars.claw.set(DoubleSolenoid.Value.kReverse);
        }

        if (vars.driveControl.getRawButton(3)) {
          timer2.reset();
          timer2.start();
          vars.trapDoor.set(DoubleSolenoid.Value.kReverse);
        } else {
          if (timer2.get() < 0.25) {
            vars.trapDoor.set(DoubleSolenoid.Value.kForward);
          } else {
            vars.trapDoor.set(DoubleSolenoid.Value.kOff);
          }
        }TODO: Enable*/

        // Drive with controller
        double ySpeed = func.controlCurve(vars.driveControl.getRawAxis(vars.yAxis));
        double xSpeed = func.controlCurve(vars.driveControl.getRawAxis(vars.xAxis)) * -1;
        double zRotation = func.controlCurve(vars.driveControl.getRawAxis(vars.zAxis));
        //vars.drive.driveCartesian(ySpeed, xSpeed, zRotation);
      }
    }
  }
}