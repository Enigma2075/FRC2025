package frc.robot.subsystems;

public class ElevatorConst {
    public static final int kMotorFrontId = 1;
    public static final int kMotorBackId = 2;

    public static final double kHeightOffset= 0;
    public static final double kInitialHeight = 0;

    public static final double kGearRatio = 4.0;
    public static final double kSpoolRadius = 1.5/2.0;
    public static final double kSpoolCircumference = 2 * Math.PI * kSpoolRadius;
    public static final double kRotationToInches = kSpoolCircumference / kGearRatio;

    public static final Elevator Elevator = new Elevator();
}
