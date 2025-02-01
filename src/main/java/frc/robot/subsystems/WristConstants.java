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

    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kMotionMagicCruiseVelocity = 0;
    public static final double kMotionMagicAcceleration = 0;
    public static final double kMotionMagicJerk = 0;

}
