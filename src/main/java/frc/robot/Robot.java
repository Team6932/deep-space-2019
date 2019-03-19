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

  Timer timer = new Timer();
  Timer timer2 = new Timer();
  private double driveSpeed = 0.0;
  private double sideSpeed = 0.0;
  private double rotateSpeed = 0.0;

  private double leftUltraReading = 0;
  private double rightUltraReading = 0;

  private double rotateError = 0;

  private boolean clawOut = false;
  private boolean liftOut = false;
  private boolean lowRates = false;

  private double desiredClawPosition = 0;
  private double clawTimeNeeded = 0;
  private boolean clawMoving = false;
  private boolean clawGoingUp = false;

  @Override
  public void robotInit() {
    // Reset gyro
    vars.gyro.reset();
    vars.gyro.calibrate();

    // Let ultrasonic sensors poll automatically
    // vars.leftUltra.setAutomaticMode(true);
    // vars.rightUltra.setAutomaticMode(true);
    // vars.frontUltra.setAutomaticMode(true);

    // Enable the compressor
    vars.compressor.setClosedLoopControl(true);
  }

  /*
   * @Override public void robotPeriodic() { // Ultrasonic drive system if
   * (vars.leftUltra.isRangeValid()) { leftUltraReading = (int)
   * vars.leftUltra.getRangeMM(); } else { leftUltraReading -= 5; } if
   * (vars.rightUltra.isRangeValid()) { rightUltraReading = (int)
   * vars.rightUltra.getRangeMM(); } else { rightUltraReading -= 5; } /* if
   * (vars.frontUltra.isRangeValid()) { frontUltraReading = (int)
   * vars.frontUltra.getRangeMM(); }
   *
   * // System.out.println(leftUltraReading + " / " + rightUltraReading); double
   * avgReading = (leftUltraReading + rightUltraReading) / 2;
   * 
   * if (avgReading < 700) { // Rotate based on the difference between ultrasonic
   * readings rotateError = leftUltraReading - rightUltraReading; rotateSpeed =
   * (rotateError * vars.rotateP); // TODO: Fine tune this value
   * 
   * // Drive forward based on ultrasonic reading averages double forwardError =
   * avgReading - 240; driveSpeed = (forwardError * vars.ultraP); // TODO: Fine
   * tune this value
   * 
   * // System.out.println("Rotate: " + rotateSpeed + "\t/\tForward:" +
   * driveSpeed); // System.out.println(vars.leftLine.getValue() +
   * "\t\t/\t\tLeft Ultra: " + // leftUltraReading + "\t\tRight Ultra: " +
   * rightUltraReading); } else { // Do not rotate or drive if at least not 500mm
   * close rotateSpeed = 0; // driveSpeed = 0; } // System.out.println(rotateSpeed
   * + " / " + driveSpeed); sideSpeed = (vars.leftLine.getVoltage() - (5 -
   * vars.rightLine.getVoltage())) * 0.25; // System.out.println(leftUltraReading
   * + " / " + rightUltraReading + " / " + // driveSpeed); }
   */

  @Override
  public void autonomousInit() {
    // timer.reset();
    // timer.start();
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
    /*
     * if (timer.get() < 2.0) { vars.lift.set(0.2); } else if (timer.get() >= 2.0 &&
     * timer.get() < 3.0) { vars.claw.set(DoubleSolenoid.Value.kForward); } else if
     * (timer.get() >= 3.0 && timer.get() < 4.0) {
     * vars.claw.set(DoubleSolenoid.Value.kReverse); } else if (timer.get() >= 4.0
     * && timer.get() < 6.0) { vars.lift.set(-0.15); } else { vars.lift.set(0);
     * }TODO: Enable
     */
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    timer2.reset();
    timer2.start();
  }

  @Override
  public void teleopPeriodic() {
    // Drive with controller
    if (vars.driveControl.getRawButtonPressed(8)) {
      lowRates = !lowRates; // Reverse whatever's there
    }
    double ySpeed = vars.driveControl.getRawAxis(vars.yAxis);
    double xSpeed = vars.driveControl.getRawAxis(vars.xAxis) * -1;
    double zRotation = vars.driveControl.getRawAxis(vars.zAxis);
    double multiplier = 0.7;
    if (lowRates) {
      multiplier = 0.3;
    }
    vars.drive.driveCartesian(ySpeed * multiplier, xSpeed * multiplier, zRotation * multiplier);

    if (vars.driveControl.getRawButtonPressed(1)) {
      clawOut = !clawOut; // Reverse whatever's there
    }
    if (clawOut) {
      vars.claw.set(DoubleSolenoid.Value.kForward);
    } else {
      vars.claw.set(DoubleSolenoid.Value.kReverse);
    }

    if (vars.driveControl.getRawButtonPressed(3)) {
      liftOut = !liftOut; // Reverse whatever's there
    }
    if (liftOut) {
      vars.trapDoor.set(DoubleSolenoid.Value.kForward);
    } else {
      vars.trapDoor.set(DoubleSolenoid.Value.kReverse);
    }

    /*
     * if (vars.driveControl.getRawButtonPressed(2)) { if (!clawMoving) {
     * desiredClawPosition -= 1; clawMoving = true; clawGoingUp = false;
     * clawTimeNeeded = timer2.get() + 0.5; if (desiredClawPosition < 0) {
     * desiredClawPosition = 0; clawMoving = false; clawTimeNeeded = 0; } } } else
     * if (vars.driveControl.getRawButtonPressed(4)) { if (!clawMoving) {
     * desiredClawPosition += 1; clawMoving = true; clawTimeNeeded = timer2.get() +
     * 0.5; clawGoingUp = true; if (desiredClawPosition < 2) { desiredClawPosition =
     * 2; clawMoving = false; clawTimeNeeded = 0; } } }
     * 
     * if (!clawMoving) { clawTimeNeeded = 0; }
     * 
     * if (clawTimeNeeded > timer2.get()) { if (clawGoingUp) { vars.lift.set(0.35);
     * } else { vars.lift.set(-0.30); } } else { vars.lift.set(0.1); clawMoving =
     * false; }
     * 
     * System.out.println(desiredClawPosition + " / " + clawTimeNeeded + " / " +
     * timer2.get());
     */

    if (vars.driveControl.getRawButton(2)) {
      vars.lift.set(-0.30);
    } else if (vars.driveControl.getRawButton(4)) {
      vars.lift.set(0.30);
    } else {
      vars.lift.set(0.1);
    }
  }
}