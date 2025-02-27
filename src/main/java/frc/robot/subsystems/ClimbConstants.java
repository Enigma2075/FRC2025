package frc.robot.subsystems;

public class ClimbConstants {
    public static final int kBackId = 18;
    public static final int kFrontId = 17;

    public static final int kLatchPort = 0;

    public static final double kGearRatio = 3.75;

    public static final double kG = 0;
    public static final double kS = 0.0146484375;
    public static final double kV = 0.01;
    public static final double kA = 0.001;
    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kFalconRPS = 6380.0 / 60.0;
    public static final double kMotionMagicCruiseVelocity = kFalconRPS * .8;
    public static final double kMotionMagicAcceleration = (kMotionMagicCruiseVelocity)/.5;
    public static final double kMotionMagicJerk = 0;
}
