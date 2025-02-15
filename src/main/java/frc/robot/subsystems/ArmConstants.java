package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.util.Utils;

public class ArmConstants {
    public static final int kMotorId = 11;

    public static final int kEncoderId = 5;

    public static final double kArmLength = 0;

    public static final double kGearRatio1 = 56/12;
    public static final double kGearRatio2 = 56/18;
    public static final double kGearRatio3 = 68/18;

    public static final double kGearRatio = kGearRatio1 * kGearRatio2 * kGearRatio3;

    public static final double kG = 0.8;
    public static final double kS = 0.6;
    public static final double kV = 2.0;
    public static final double kA = 1.0;
    public static final double kP = 50;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kKraken60RPS = 6000.0 / 60.0;
    public static final double kMaxArmRPS = kKraken60RPS / kGearRatio;
    public static final double kMotionMagicCruiseVelocity = kMaxArmRPS * .5;

    public static final double kMotionMagicAcceleration = (kMaxArmRPS * .5)/.65;
    public static final double kMotionMagicJerk = 0;

    public static final double kMagnetOffset = RobotConstants.kPracticeBot ? 0.3125 : 0;
}
