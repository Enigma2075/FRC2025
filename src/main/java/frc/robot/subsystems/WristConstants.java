package frc.robot.subsystems;

public class WristConstants {
    public static final int kMotorId = 1;

    public static final int kEncoderId = 0;

    public static final double kWristToEndEffectorLength = 0;

    public static final double kGearRatio1 = 56/12;
    public static final double kGearRatio2 = 56/24;
    public static final double kGearRatio3 = 64/24;

    public static final double kGearRatio = kGearRatio1 * kGearRatio2 * kGearRatio3;

    public static final Wrist Wrist = new Wrist();
}
