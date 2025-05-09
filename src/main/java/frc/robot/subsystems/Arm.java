package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.util.Utils;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class Arm extends SubsystemIO{

    public enum ControlMode{
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_Motor;
    private CANcoder m_Encoder;
    private CANrange m_BargeSensor;

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage m_PositionRequest = new MotionMagicVoltage(0).withSlot(0);

    private final VoltageOut m_SysIdRequest = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Volts.of(3).per(Seconds.of(1)),
                    Volts.of(3),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        m_Motor.setControl(m_SysIdRequest.withOutput(m_SysIdRequest.withOutput(volts.in(Volts)).Output));
                        //System.out.println(volts.in(Volts));
                    },
                    (SysIdRoutineLog log) -> log.motor("Arm").voltage(m_Motor.getMotorVoltage().getValue()).angularPosition(m_Motor.getPosition().getValue()).angularVelocity(m_Motor.getVelocity().getValue()),
                    (Subsystem)this));

    public Arm(){
        m_Encoder = new CANcoder(ArmConstants.kEncoderId, RobotConstants.kCanivoreBusName);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ArmConstants.kDiscontinuityPoint;
        if(RobotConstants.kPracticeBot) {
            encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }
        else {
            encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }

        encoderConfig.MagnetSensor.MagnetOffset = ArmConstants.kMagnetOffset;
        
        m_Encoder.getConfigurator().apply(encoderConfig);

        m_Motor = new TalonFX(ArmConstants.kMotorId, RobotConstants.kCanivoreBusName);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Feedback.FeedbackRemoteSensorID = m_Encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        motorConfig.Feedback.RotorToSensorRatio = ArmConstants.kGearRatio;

        Slot0Configs slot0Configs = motorConfig.Slot0;
        slot0Configs.kG = 0;
        slot0Configs.kS = ArmConstants.kS;
        slot0Configs.kV = ArmConstants.kV;
        slot0Configs.kA = ArmConstants.kA;
        slot0Configs.kP = ArmConstants.kP;
        slot0Configs.kI = ArmConstants.kI;
        slot0Configs.kD = ArmConstants.kD;
        
        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.kMotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.kMotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ArmConstants.kMotionMagicJerk;
       
        motorConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerLimit(.75)
            .withSupplyCurrentLimitEnable(true)
        );

        m_Motor.getConfigurator().apply(motorConfig);

        m_Motor.setControl(m_OutputRequest.withOutput(0));

        m_BargeSensor = new CANrange(ArmConstants.kBargeSensorId, RobotConstants.kCanivoreBusName);

        CANrangeConfiguration bargeSensorConfig = new CANrangeConfiguration();

        bargeSensorConfig.FovParams.FOVCenterX = 0;
        bargeSensorConfig.FovParams.FOVCenterY = 0;
        bargeSensorConfig.FovParams.FOVRangeX = 7;
        bargeSensorConfig.FovParams.FOVRangeY = 7;
        bargeSensorConfig.ProximityParams.ProximityThreshold = 1.5;

        m_BargeSensor.getConfigurator().apply(bargeSensorConfig);
    }

    public static class PeriodicIO {
        public ControlMode controlMode = ControlMode.OUTPUT;
        double enc = 0;

        public double CurrentAngle = 0;
        public double targetAngle = 0;

        public double targetOutput = 0; 

        public double distanceToBarge = 0;
        public boolean seeBarge = false;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setOutput(double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    public void setDegrees(double degrees){
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.targetAngle = Math.toRadians(degrees);
    }

    public double getCurrentAngle() {
        return m_PeriodicIO.CurrentAngle;
    }

    private double convertPositionToAngle(double position) {
        return position * 2 * Math.PI ;
    }

    private double convertAngleToPosition(double angle) {
        return angle / (2 * Math.PI);
    }

    public boolean isAtPosition(double degrees) {
        double error = Math.toDegrees(m_PeriodicIO.CurrentAngle) - degrees;
        return Math.abs(error) < 5;
    }

    public boolean seeBarge() {
        return m_PeriodicIO.seeBarge;
    }

    public double bargeError() {
        return m_PeriodicIO.distanceToBarge - 63;
    }

    public boolean isAtBarge(){
        return seeBarge() && bargeError() <= 0; //physical measurement = 
    }

    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.quasistatic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.dynamic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command testCommand(Supplier<Double> outputPercent) {
        return run(() -> {
                setOutput(outputPercent.get());
        });
    }

    public Command setPositionCommand(double degrees){
        return run(() -> {
            m_PeriodicIO.targetAngle = Math.toRadians(degrees);
            m_PeriodicIO.controlMode = ControlMode.POSITION;
        });
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Arm/CurrentDegrees", Math.toDegrees(m_PeriodicIO.CurrentAngle));
        SmartDashboard.putNumber("Arm/TargetDegrees", Math.toDegrees(m_PeriodicIO.targetAngle));

        SignalLogger.writeDouble("Arm/CurrentDegrees", Math.toDegrees(m_PeriodicIO.CurrentAngle));
        SignalLogger.writeDouble("Arm/TargetDegrees", Math.toDegrees(m_PeriodicIO.targetAngle));
    }

    @Override
    public void readPeriodicInputs(){
        m_PeriodicIO.CurrentAngle = convertPositionToAngle(m_Motor.getPosition().getValueAsDouble());
        m_PeriodicIO.distanceToBarge = m_BargeSensor.getDistance().getValue().in(Centimeters);
        m_PeriodicIO.seeBarge = m_BargeSensor.getIsDetected().getValue();
    }

    @Override
    public void writePeriodicOutputs(){
        switch(m_PeriodicIO.controlMode){
            case OUTPUT:
                m_Motor.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                break;
            case POSITION:
                if(DriverStation.isAutonomousEnabled() && Robot.RobotContainer.elevator.isAtPositionAuto()) {
                    double sign = m_PeriodicIO.targetAngle < Math.PI/2 ? -1 : -1;
                    double targetAngle = m_PeriodicIO.targetAngle + ((sign * Utils.getValue(0, Math.toRadians(3.4))));
    
                    m_Motor.setControl(m_PositionRequest.withPosition(convertAngleToPosition(targetAngle)).withFeedForward(Math.cos(m_PeriodicIO.CurrentAngle) * ArmConstants.kG));    
                }
                else if(Robot.RobotContainer.elevator.isAtPosition()) {
                    double sign = m_PeriodicIO.targetAngle < Math.PI/2 ? -1 : -1;
                    double targetAngle = m_PeriodicIO.targetAngle + ((sign * Utils.getValue(0, Math.toRadians(3.4))));
    
                    m_Motor.setControl(m_PositionRequest.withPosition(convertAngleToPosition(targetAngle)).withFeedForward(Math.cos(m_PeriodicIO.CurrentAngle) * ArmConstants.kG));    
                }
                else {
                    m_Motor.setControl(m_PositionRequest.withPosition(convertAngleToPosition(Math.toRadians(90))).withFeedForward(Math.cos(m_PeriodicIO.CurrentAngle) * ArmConstants.kG));
                }
                break;
            case SYSID:

                break;
            default:

                break;
        }
    }
}
