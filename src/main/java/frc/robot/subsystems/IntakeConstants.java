package frc.robot.subsystems;

public class IntakeConstants {
    public static final int kPivotId = 15;
    public static final int kRollerId = 16;

    public static final double kGearRatio1 = 9.0/1.0;
    public static final double kGearRatio2 = 38.0/18.0;
    public static final double kGearRatio = kGearRatio1 * kGearRatio2;
    
    public static final double kG = 0.5;
    public static final double kS = 0.0146484375;
    public static final double kV = 0.01;
    public static final double kA = 0.001;
    public static final double kP = 0.0001;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kZeroOffset = 7.772461;

    public static final double kFalconRPS = 6380.0 / 60.0;
    public static final double kMaxRPS = kFalconRPS / kGearRatio;
    public static final double kMotionMagicCruiseVelocity = kMaxRPS * .2;
    public static final double kMotionMagicAcceleration = (kMotionMagicCruiseVelocity)/1.772;
    public static final double kMotionMagicJerk = 0;
}
