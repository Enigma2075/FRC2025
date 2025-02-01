package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ElevatorConst {
    public static final int kMotorFrontId = 1;
    public static final int kMotorBackId = 2;

    public static final double kHeightOffset= 0;
    public static final double kInitialHeight = 0;

    public static final double kGearRatio = 4.0;
    public static final double kSpoolRadius = 1.5/2.0;
    public static final double kSpoolCircumference = 2 * Math.PI * kSpoolRadius;
    public static final double kRotationToInches = kSpoolCircumference / kGearRatio;

    public static final Elevator Elevator = new Elevator(ArmConstants.Arm, WristConstants.Wrist, ClawConstants.Claw);

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
