package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
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
    public Ultrasonic rightUltra = new Ultrasonic(1, 0);
    public Ultrasonic leftUltra = new Ultrasonic(3, 2);
    public Ultrasonic frontUltra = new Ultrasonic(4, 5);

    // Line sensors
    public AnalogInput leftLine = new AnalogInput(0);
    public AnalogInput rightLine = new AnalogInput(1);

    // Pneumatics
    int pcmID = 1;
    Compressor compressor = new Compressor(pcmID);
    DoubleSolenoid claw = new DoubleSolenoid(pcmID, 1, 0);
    DoubleSolenoid trapDoor = new DoubleSolenoid(pcmID, 5, 4);

    // Joystick/Controller
    public Joystick joystick = new Joystick(0);
    public Joystick controller = new Joystick(1);

    // Controller Configuration
    public Joystick driveControl = controller;
    public int yAxis = 0;
    public int xAxis = 1;
    public int zAxis = 2;

    // Lift logic
    public boolean lifting = false; // 0 do nothing, 1-3 positions
    public double liftP = 0.003;
    public double liftI = 0;

    // Alignment logic
    public double rotateP = -0.002;
    public double ultraP = 0.002;

    // Drive logic
    public double driveP = 0.002;
    public double driveI = 0;

    public static InstanceStorage getInstance() {
        return instance;
    }

}
