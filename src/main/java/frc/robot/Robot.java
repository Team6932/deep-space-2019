/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  private InstanceStorage vars = InstanceStorage.getInstance();
  private CustomFunctions func = CustomFunctions.getInstance();
  private NetworkTableEntry m_maxSpeed;

  private VisionThread visionThread;
  private double centerX = 0.0;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 180;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    // Reset gyro
    vars.gyro.calibrate();
    vars.gyro.reset();
    m_maxSpeed = Shuffleboard.getTab("Configuration").add("Max Speed", 1).withWidget("Number Slider").withPosition(1, 1)
        .withSize(2, 1).getEntry();

    // Add the tank drive and encoders to a 'Drivebase' tab
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Tank Drive", "derp");
    // Put both encoders in a list layout
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    encoders.add("Left Encoder", "derp2");
    encoders.add("Right Encoder", "m_rightEncoder");

    // Add the elevator motor and potentiometer to an 'Elevator' tab
    ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    elevatorTab.add("Motor", "derp");
    elevatorTab.add("Potentiometer", "derp2");

    /*UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new BallPipeline(), pipeline -> {
      if (!pipeline.findContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
        }
      } else {
        synchronized (imgLock) {
          centerX = IMG_WIDTH / 2;
        }
      }
    });
    visionThread.start();*/
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
    vars.drive.arcadeDrive(0.0, turn);*/
  }

  @Override
  public void teleopPeriodic() {
      // Drive robot
      double ySpeed = func.ratioValue(vars.driveControl.getRawAxis(vars.yAxis));
      double xSpeed = func.ratioValue(vars.driveControl.getRawAxis(vars.xAxis)) * -1;
      double zRotation = func.ratioValue(vars.driveControl.getRawAxis(vars.zAxis));
      vars.drive.driveCartesian(ySpeed, xSpeed, zRotation);
  }
}