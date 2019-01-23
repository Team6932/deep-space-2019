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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  private final DifferentialDrive m_tankDrive = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final PWMVictorSPX m_elevatorMotor = new PWMVictorSPX(2);
  private final AnalogPotentiometer m_elevatorPot = new AnalogPotentiometer(0);
  private NetworkTableEntry m_maxSpeed;

  private VisionThread visionThread;
  private double centerX = 0.0;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    m_maxSpeed = Shuffleboard.getTab("Configuration").add("Max Speed", 1).withWidget("Number Slider").withPosition(1, 1)
        .withSize(2, 1).getEntry();

    // Add the tank drive and encoders to a 'Drivebase' tab
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Tank Drive", m_tankDrive);
    // Put both encoders in a list layout
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    encoders.add("Left Encoder", m_leftEncoder);
    encoders.add("Right Encoder", m_rightEncoder);

    // Add the elevator motor and potentiometer to an 'Elevator' tab
    ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    elevatorTab.add("Motor", m_elevatorMotor);
    elevatorTab.add("Potentiometer", m_elevatorPot);

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new MMPipeline(), pipeline -> {
      if (!pipeline.findContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
        }
      }
    });
    visionThread.start();
  }

  @Override
  public void autonomousInit() {
    // Read the value of the 'max speed' widget from the dashboard
    m_tankDrive.setMaxOutput(m_maxSpeed.getDouble(1.0));
  }

  @Override
  public void autonomousPeriodic() {
    double centerX;
    synchronized (imgLock) {
      centerX = this.centerX;
    }
    double turn = centerX - (IMG_WIDTH / 2);
    m_tankDrive.arcadeDrive(-0.6, turn * 0.005);
  }

}
