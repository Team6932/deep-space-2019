package frc.robot;

public class CustomFunctions {
    private static CustomFunctions instance = new CustomFunctions();
    private static RobotHW vars = RobotHW.getInstance();
    private double targetHeading = 0;
    private double gyroError = 0;
    private double gyroErrorSum = 0;

    public static CustomFunctions getInstance() {
        return instance;
    }

    public void gyroCorrectedDrive(double ySpeed, double xSpeed, double zRotation) {
        targetHeading += zRotation;
        double currentHeading = vars.gyro.getAngle();
        gyroError = targetHeading - currentHeading;
        gyroErrorSum += gyroError;
        vars.drive.driveCartesian(ySpeed, xSpeed, (gyroError * vars.driveP) + (gyroErrorSum * vars.driveI));
    }
}