package frc.robot.subsystems;

public class ClimbConstants {
    public static final int kBackId = 14;
    public static final int kFrontId = 2;

    public static final double kGearRatio = 3.75;

    public static final double kG = 0;
    public static final double kS = 0.015;
    public static final double kV = 0.001;
    public static final double kA = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kMotionMagicCruiseVelocity = 6380 * .5;
    public static final double kMotionMagicAcceleration = (6380 * .5)/3;
    public static final double kMotionMagicJerk = 0;
    

    public static final Climb Climb = new Climb();
}
