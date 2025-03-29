package frc.robot.subsystems;

public class IntakeConstants {
    public static final int kPivotId = 15;
    public static final int kRollerId = 16;
    public static final int kSensorId = 1;

    public static final double kMaxRange = 30;
    public static final double kMinRange = 8;

    public static final double kGearRatio1 = 7.0/1.0;
    public static final double kGearRatio2 = 3.0/1.0;
    public static final double kGearRatio3 = 50.0/34.0;
    public static final double kGearRatio = kGearRatio1 * kGearRatio2 * kGearRatio3;
    
    public static final double kG = 0.55;
    public static final double kS = 0.05; //0.50892;
    public static final double kV = 0.12027;
    public static final double kA = 0.0058869;
    public static final double kP = 12.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kZeroOffset = 9.901;

    public static final double kFalconRPS = 6380.0 / 60.0;
    public static final double kMaxRPS = kFalconRPS / kGearRatio;
    public static final double kMotionMagicCruiseVelocity = kFalconRPS * .9;
    public static final double kMotionMagicAcceleration = (kMotionMagicCruiseVelocity)/.85;
    public static final double kMotionMagicJerk = 0;
}
