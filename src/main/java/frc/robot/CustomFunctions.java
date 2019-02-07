package frc.robot;

public class CustomFunctions {
    private static CustomFunctions instance = new CustomFunctions();
    private static InstanceStorage vars = InstanceStorage.getInstance();

    public static CustomFunctions getInstance() {
        return instance;
    }

    // Non-blocking functions

    // Adjust joystick values for slider
    public double controlCurve(double val) {
        //return val * (((vars.driveControl.getRawAxis(vars.ratioAxis) * -1) + 1) / 2);
        return val * 0.275;
    }
}