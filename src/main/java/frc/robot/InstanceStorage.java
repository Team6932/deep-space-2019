package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
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
    public Spark lift = new Spark(5);
    /*public Spark leftDrive = new Spark(0);
    public Spark rightDrive = new Spark(1);
    public DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);*/

    // Ultrasonic sensors
    public Ultrasonic leftUltra = new Ultrasonic(1, 0);
    public Ultrasonic rightUltra = new Ultrasonic(3, 2);
    public Ultrasonic frontUltra = new Ultrasonic(5, 4);

    // Line sensors
    //public AnalogInput leftLine = new AnalogInput(0);

    // Pneumatics
    Compressor compressor = new Compressor(1);
    DoubleSolenoid claw = new DoubleSolenoid(1, 1, 0);

    // Joystick/Controller
    public Joystick joystick = new Joystick(0);
    public Joystick controller = new Joystick(1);

    // Controller Configuration
    public Joystick driveControl = controller;
    public int yAxis = 2;
    public int xAxis = 3;
    public int zAxis = 0;

    // Lift logic
    public boolean lifting = true; // 0 do nothing, 1-3 positions
    public double targetLiftHeight = 0;
    public double liftError = 0;
    public double liftErrorSum = 0;
    public double liftP = 0.003;
    public double liftI = 0;
    public double liftD = 0;

    public static InstanceStorage getInstance() {
        return instance;
    }

}
