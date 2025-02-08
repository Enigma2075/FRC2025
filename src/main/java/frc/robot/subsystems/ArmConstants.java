package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ArmConstants {
    public static final int kMotorId = 11;

    public static final int kEncoderId = 5;

    public static final double kArmLength = 0;

    public static final double kGearRatio1 = 56/12;
    public static final double kGearRatio2 = 56/18;
    public static final double kGearRatio3 = 68/16;

    public static final double kGearRatio = kGearRatio1 * kGearRatio2 * kGearRatio3;

    //public static final Arm Arm = new Arm();    

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
}
