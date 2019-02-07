package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class InstanceStorage {

    private static InstanceStorage instance = new InstanceStorage();

    // Sensors
    Gyro gyro = new ADXRS450_Gyro();

    // Motor controllers
    public PWMVictorSPX frontLeft = new PWMVictorSPX(0);
    public PWMVictorSPX frontRight = new PWMVictorSPX(1);
    public PWMVictorSPX rearRight = new PWMVictorSPX(3);
    public PWMVictorSPX rearLeft = new PWMVictorSPX(2);
    public MecanumDrive drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    /*public Spark leftDrive = new Spark(0);
    public Spark rightDrive = new Spark(1);
    public DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);*/

    // Joystick/Controller
    public Joystick joystick = new Joystick(0);
    public Joystick controller = new Joystick(1);

    // Controller Configuration
    public Joystick driveControl = controller;
    public int yAxis = 2;
    public int xAxis = 3;
    public int zAxis = 0;

    // Motor configuration
    public double driveKp = 0.1;
    public double turnKp = 0.3;
    public double metersPerSecond = 1.5;
    public double driveSpeed = 0.75;
    public double turnSpeed = 0.75;
    public double turnTolerance = 4;
    public double turnSensitivity = 4;

    // Global constants
    public final int RIGHT = 1;
    public final int LEFT = -1;

    public static InstanceStorage getInstance() {
        return instance;
    }

}
