package frc.robot.subsystems;

public class WristConstants {
    public static final int kMotorId = 12;
    public static final int kEncoderId = 6;

    public static final double kWristToEndEffectorLength = 0;

    public static final double kGearRatio1 = 56.0/12.0;
    public static final double kGearRatio2 = 56.0/24.0;
    public static final double kGearRatio3 = 64.0/24.0;

    public static final double kGearRatio = kGearRatio1 * kGearRatio2 * kGearRatio3;

    public static final double kDiscontinuityPoint = .45836624;

    public static final double kG = 0.2;
    public static final double kS = 0.35;
    public static final double kV = 1.55;
    public static final double kA = 0.02;
    public static final double kP = 35.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kKraken44RPS = 7530.0 / 60.0;
    public static final double kMaxRPS = kKraken44RPS / kGearRatio;
    public static final double kMotionMagicCruiseVelocity = kMaxRPS * .5;
    public static final double kMotionMagicAcceleration = (kMotionMagicCruiseVelocity)/.1;
    public static final double kMotionMagicJerk = 0;

    public static final double kMagnetOffset = RobotConstants.kPracticeBot ? -0.118408203125 : 0.30615234375;
}
