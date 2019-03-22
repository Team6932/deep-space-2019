/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends TimedRobot {
  private RobotHardware robot = RobotHardware.getInstance();

  Timer timer = new Timer();
  Timer timer2 = new Timer();

  private boolean clawOut = false;
  private boolean liftOut = false;
  private boolean lowRates = false;

  private NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

  @Override
  public void robotInit() {
    // Reset gyro
    robot.gyro.reset();
    robot.gyro.calibrate();

    // Let ultrasonic sensors poll automatically
    // vars.leftUltra.setAutomaticMode(true);
    // vars.rightUltra.setAutomaticMode(true);
    // vars.frontUltra.setAutomaticMode(true);

    // Enable the compressor
    robot.compressor.setClosedLoopControl(true);

    // Clear any sticky faults
    robot.compressor.clearAllPCMStickyFaults();
    //robot.pdp.clearStickyFaults();
  }

  @Override
  public void robotPeriodic() {
    NetworkTable dsInfo = ntinst.getTable("dsInfo");
    dsInfo.getEntry("lowRates").setBoolean(lowRates);
    dsInfo.getEntry("clawOut").setBoolean(clawOut);
    dsInfo.getEntry("liftOut").setBoolean(liftOut);
    double robotLineValue = (5 - robot.rightLine.getVoltage()) - (robot.leftLine.getVoltage());
    double robotLineAvg = (5 - robot.rightLine.getVoltage()) + (robot.leftLine.getVoltage()) / 2;
    dsInfo.getEntry("lineSensor").setDouble(robotLineValue);
    dsInfo.getEntry("lineAvg").setDouble(robotLineAvg);
    dsInfo.getEntry("onLine").setBoolean((Math.abs(robotLineValue - 0.16) < 0.4) && robotLineAvg < 5);
    dsInfo.getEntry("liftSensor").setBoolean(robot.liftSensor.get());
    dsInfo.getEntry("compressorEnabled").setBoolean(robot.compressor.enabled());
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
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
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
    if (robot.driveControl.getRawButtonPressed(8)) {
      lowRates = !lowRates; // Reverse whatever's there
    }
    double ySpeed = robot.driveControl.getRawAxis(robot.yAxis);
    double xSpeed = robot.driveControl.getRawAxis(robot.xAxis) * -1;
    double zRotation = robot.driveControl.getRawAxis(robot.zAxis);
    double multiplier = 0.9;
    if (lowRates) {
      multiplier = 0.3;
    }
    robot.drive.driveCartesian(ySpeed * multiplier, xSpeed * multiplier, zRotation * multiplier);

    if (robot.driveControl.getRawButtonPressed(1)) {
      clawOut = !clawOut; // Reverse whatever's there
    }
    if (clawOut) {
      robot.claw.set(DoubleSolenoid.Value.kForward);
    } else {
      robot.claw.set(DoubleSolenoid.Value.kReverse);
    }

    if (robot.driveControl.getRawButtonPressed(3)) {
      liftOut = !liftOut; // Reverse whatever's there
    }
    if (liftOut) {
      robot.trapDoor.set(DoubleSolenoid.Value.kForward);
    } else {
      robot.trapDoor.set(DoubleSolenoid.Value.kReverse);
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

    if (robot.driveControl.getPOV() == 180 && robot.liftSensor.get()) {
        robot.lift.set(-0.30);
    } else if (robot.driveControl.getPOV() == 0) {
      robot.lift.set(0.30);
    } else {
      robot.lift.set(0.13);
    }
  }
}