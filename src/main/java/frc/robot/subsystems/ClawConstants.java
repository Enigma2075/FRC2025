package frc.robot.subsystems;

public class ClawConstants {
    public static final int kCoralId = 13;
    public static final int kAlgaeId = 14;

    public static final int kCoralSensorId = 2;
    public static final int kAlgaeSensorId = 3;

    public static final double kG = 0;
    public static final double kS = 0.0146484375;
    public static final double kV = 0.01;
    public static final double kA = 0.001;
    public static final double kP = 0.0001;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kFalconRPS = 6380.0 / 60.0;
    public static final double kMotionMagicCruiseVelocity = kFalconRPS * .5;
    public static final double kMotionMagicAcceleration = (kFalconRPS * .5)/1.772;
    public static final double kMotionMagicJerk = 0;
    
    public static final double kCoralDistanceThreshold = 10;
    public static final double kAlgaeDistanceThreshold = 8;

    }

