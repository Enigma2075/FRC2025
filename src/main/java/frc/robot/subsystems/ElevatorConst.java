package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.util.Utils;

public class ElevatorConst {
    public static final int kMotorFrontId = 9;
    public static final int kMotorBackId = 10;

    public static final double kHeightOffset= 7;
    public static final double kInitialHeight = 0;

    public static final double kMaxHeight = 67;

    public static final double kMaxHeightValue = Utils.getValue(57.5, 59.93);
    
    public static final double kGearRatio = 3.5;
    public static final double kSpoolRadius = 1.484/2.0;
    public static final double kSpoolCircumference = 2 * Math.PI * kSpoolRadius;
    public static final double kRotationToInches = kSpoolCircumference / kGearRatio;

    public static final double kErrorCorrectionRatio = (64.5 - Utils.getValue(57.5, 59.93)) / 64.5;

    public static final double kG = 0.41;
    public static final double kS = 0.094003;
    public static final double kV = 0.13099;
    public static final double kA = 0.0019951;
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kFalconRPS = 6380.0 / 60.0;
    public static final double kMotionMagicCruiseVelocity = kFalconRPS * .7;
    public static final double kMotionMagicAcceleration = (kFalconRPS * .7)/.5;
    public static final double kMotionMagicJerk = 0;

    // Physical Properties
    // Supply Current to hold at 30 inches = .38

}
