package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemIO{
    public enum ControlMode{
        OUTPUT,
        POSITION,
        SYSID
    }
    
    private TalonFX m_Motor;
    private CANcoder m_Encoder;

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final DynamicMotionMagicVoltage m_PositionRequest = new DynamicMotionMagicVoltage(0, WristConstants.kMotionMagicCruiseVelocity, WristConstants.kMotionMagicAcceleration, WristConstants.kMotionMagicJerk).withSlot(0);

    private final VoltageOut m_SysIdRequest = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volt.of(.8).per(Seconds),
                    Volts.of(2),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        m_Motor.setControl(m_SysIdRequest.withOutput(m_SysIdRequest.withOutput(volts.in(Volts)).Output));
                        //System.out.println(volts.in(Volts));
                    },
                    (SysIdRoutineLog log) -> log.motor("Wrist").voltage(m_Motor.getMotorVoltage().getValue()).angularPosition(m_Motor.getPosition().getValue()).angularVelocity(m_Motor.getVelocity().getValue()),
                    (Subsystem)this));

    public Wrist(){
        m_Encoder = new CANcoder(WristConstants.kEncoderId, RobotConstants.kCanivoreBusName);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = WristConstants.kDiscontinuityPoint;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = WristConstants.kMagnetOffset;
        
        m_Encoder.getConfigurator().apply(encoderConfig);

        m_Motor = new TalonFX(WristConstants.kMotorId,RobotConstants.kCanivoreBusName);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Feedback.FeedbackRemoteSensorID = m_Encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.kGearRatio;

        Slot0Configs slot0Configs = motorConfig.Slot0;
        slot0Configs.kS = WristConstants.kS;
        slot0Configs.kV = WristConstants.kV;
        slot0Configs.kA = WristConstants.kA;
        slot0Configs.kP = WristConstants.kP;
        slot0Configs.kI = WristConstants.kI;
        slot0Configs.kD = WristConstants.kD;

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = WristConstants.kMotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = WristConstants.kMotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = 0;


        m_Motor.getConfigurator().apply(motorConfig);
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;

        public boolean overrideVelocity = false;

        public double CurrentAngle = 0;
        public double TargetAngle = 0;
    
        public double targetOutput = 0;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setOutput (double output) {
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    public void setDegrees(double degrees){
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.TargetAngle = Math.toRadians(degrees);
    }

    public void setOverrideVelocity(boolean override) {
        m_PeriodicIO.overrideVelocity = override;
    } 

    private double convertPositionToAngle(double position) {
        return position * (2 * Math.PI);
    }

    private double convertAngleToPosition(double angle) {
        return angle / (2 * Math.PI);
    }

    public boolean isAtPosition(double degrees) {
        double error = Math.toDegrees(m_PeriodicIO.CurrentAngle) - degrees;
        return Math.abs(error) < 5;
    }

    private double getGravityOffset() {
        return Math.cos(Robot.RobotContainer.arm.getCurrentAngle() + m_PeriodicIO.CurrentAngle) * WristConstants.kG;
    }

    //rad(165)/2 pi = .458

    public Command testCommand(Supplier<Double> outputPercent) {
        return run(() -> {
                setOutput(outputPercent.get());
        });
    }

    public Command setTestPosition(double degrees){
        return run(() -> {
            setDegrees(degrees);
        });
    }

    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.quasistatic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.dynamic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
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
        SmartDashboard.putNumber("Wrist/CurrentDegrees", Math.toDegrees(m_PeriodicIO.CurrentAngle));
        SmartDashboard.putNumber("Wrist/TargetDegrees", Math.toDegrees(m_PeriodicIO.TargetAngle));

        SignalLogger.writeDouble("Wrist/CurrentDegrees", Math.toDegrees(m_PeriodicIO.CurrentAngle));
        SignalLogger.writeDouble("Wrist/TargetDegrees", Math.toDegrees(m_PeriodicIO.TargetAngle));
    }

    @Override
    public void readPeriodicInputs(){
        m_PeriodicIO.CurrentAngle = convertPositionToAngle(m_Motor.getPosition().getValueAsDouble());
    }

    @Override
    public void writePeriodicOutputs() {
           switch(m_PeriodicIO.controlMode){
                case OUTPUT:
                    m_Motor.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                    break;
                case POSITION:
                    if(m_PeriodicIO.overrideVelocity) {
                        
                        m_Motor.setControl(m_PositionRequest.withPosition(convertAngleToPosition(m_PeriodicIO.TargetAngle)).withVelocity(WristConstants.kMotionMagicCruiseVelocity * .5).withFeedForward(getGravityOffset()));
                    }
                    else {
                        m_Motor.setControl(m_PositionRequest.withPosition(convertAngleToPosition(m_PeriodicIO.TargetAngle)).withFeedForward(getGravityOffset()));
                    }
                    break;
                case SYSID:
                
                    break;
                default:

                    break;
            }
    }
}    


