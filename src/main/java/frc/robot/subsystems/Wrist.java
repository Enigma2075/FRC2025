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
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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

public class Wrist extends SubsystemIO{
    public enum ControlMode{
        OUTPUT,
        POSITION,
        SYSID
    }
    
    private TalonFX m_Motor;
    private CANcoder m_Encoder;

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final MotionMagicDutyCycle m_PositionRequest = new MotionMagicDutyCycle(0).withSlot(0);

    private final VoltageOut m_SysIdRequest = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volt.of(2).per(Seconds),
                    Volts.of(4.5),
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
        
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .45836624;
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
        slot0Configs.kG = IntakeConstants.kG;
        slot0Configs.kS = IntakeConstants.kS;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicAcceleration = 0;
        motionMagicConfigs.MotionMagicJerk = 0;

        m_Motor.getConfigurator().apply(motorConfig);
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;

        public double CurrentAngle = 0;
        public double TargetAngle = 0;
    
        public double targetOutput = 0;
    }

    public void setOutput (double output) {
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    public void setDegrees(double degrees){
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.TargetAngle = Math.toRadians(degrees);
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToAngle(double position) {
        return position * 2 * Math.PI ;
    }

    private double convertAngleToPosition(double angle) {
        return angle / 2 * Math.PI;
    }

    //rad(165)/2 pi = .458

    public Command testCommand(Supplier<Double> outputPercent) {
        return new Command() {
            @Override
            public void execute() {
                setOutput(outputPercent.get() * 12.0);
            }
        };
    }

    public Command setTestPosition(double degrees){
        return run(() -> {
            setDegrees(degrees);
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
        SmartDashboard.putNumber("Wrist/CurrentAngle", Math.toDegrees(m_PeriodicIO.CurrentAngle));
        SmartDashboard.putNumber("Wrist/TargetAngle", Math.toDegrees(m_PeriodicIO.TargetAngle));

        SignalLogger.writeDouble("Wrist/CurrentAngle", Math.toDegrees(m_PeriodicIO.CurrentAngle));
        SignalLogger.writeDouble("Wrist/TargetAngle", Math.toDegrees(m_PeriodicIO.TargetAngle));
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
                    m_Motor.setControl(m_PositionRequest.withPosition(convertAngleToPosition(m_PeriodicIO.TargetAngle)));
                    break;
                case SYSID:
                
                    break;
                default:

                    break;
            }
    }
}    


