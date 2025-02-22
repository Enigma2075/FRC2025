package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemIO{
    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_pivot;
    private TalonFX m_roller;

    private final DutyCycleOut m_PivotOutputRequest= new DutyCycleOut(0);
    private final MotionMagicVoltage m_PivotPositionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut m_RollerOutputRequest= new DutyCycleOut(0);

    private final VoltageOut m_SysIdRequest = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(2).per(Seconds),
                    Volts.of(4.5),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        m_pivot.setControl(m_SysIdRequest.withOutput(m_SysIdRequest.withOutput(volts.in(Volts)).Output));
                        //System.out.println(volts.in(Volts));
                    },
                    (SysIdRoutineLog log) -> log.motor("intake").voltage(m_pivot.getMotorVoltage().getValue()).angularPosition(m_pivot.getPosition().getValue()).angularVelocity(m_pivot.getVelocity().getValue()),
                    (Subsystem)this));

    public Intake() {
        m_pivot = new TalonFX(IntakeConstants.kPivotId,RobotConstants.kCanivoreBusName);
        m_roller = new TalonFX(IntakeConstants.kRollerId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        Slot0Configs slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kG = IntakeConstants.kG;
        slot0Configs.kS = IntakeConstants.kS;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;


        MotionMagicConfigs motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.kMotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.kMotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = IntakeConstants.kMotionMagicJerk;

        m_pivot.getConfigurator().apply(pivotConfigs);

        m_pivot.setPosition(IntakeConstants.kZeroOffset);

        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
        
        rollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_roller.getConfigurator().apply(rollerConfigs);
    }

    public enum PivotPositions { 
        GRABCAGE(0),
        FLOORINTAKE(34),
        CLIMBREADY(59),
        DEFAULT(94);

        public final double degrees;

        private PivotPositions(double degrees) {
            this.degrees = degrees;
        }
    } 

    public static class PeriodicIO {
        public ControlMode controlMode = ControlMode.OUTPUT;    

        PivotPositions targetPivotPosition = PivotPositions.DEFAULT;
        
        public double targetPivotOutput = 0;
        
        public double targetPivotAngle = 0;
        public double currentPivotAngle = 0;

        public double targetRollerOutput = 0;
        public double currentRollerCurrent = 0;
    }

    public void setPivotOutput (double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetPivotOutput = output;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToAngle(double position) {
        return (position / IntakeConstants.kGearRatio)  * 2 * Math.PI;
    }

    private double convertAngleToPosition(double angle) {
        return (angle * IntakeConstants.kGearRatio) / (2 * Math.PI);
    }
    
    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.quasistatic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.dynamic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command testCommand(Supplier<Double> outputPercent){
        return run(() -> {
            setPivotOutput(outputPercent.get());
        });
    }
    
    public Command setTestPosition(){
        return run(() -> {
            m_PeriodicIO.targetPivotPosition = PivotPositions.GRABCAGE;
            m_PeriodicIO.controlMode = ControlMode.POSITION;

        });
    }

    public void setPivotPosition(PivotPositions position){
        m_PeriodicIO.targetPivotPosition = position;
        m_PeriodicIO.targetPivotAngle = Math.toRadians(position.degrees);
        m_PeriodicIO.controlMode = ControlMode.POSITION;
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.currentPivotAngle = convertPositionToAngle(m_pivot.getPosition().getValueAsDouble());
        m_PeriodicIO.currentRollerCurrent = m_roller.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:   
                m_pivot.setControl(m_PivotOutputRequest.withOutput(m_PeriodicIO.targetPivotOutput));
                break;

            case POSITION:
                m_pivot.setControl(m_PivotPositionRequest.withPosition(convertAngleToPosition(m_PeriodicIO.targetPivotAngle)).withFeedForward(Math.cos(m_PeriodicIO.currentPivotAngle) * WristConstants.kG));
                break;

            case SYSID:

                break;

            default:

                break;
        }
        m_roller.setControl(m_RollerOutputRequest.withOutput(m_PeriodicIO.targetRollerOutput));
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
            SmartDashboard.putNumber("Intake/CurrentDegrees", Math.toDegrees(m_PeriodicIO.currentPivotAngle));
            SmartDashboard.putNumber("Intake/TargetDegrees", Math.toDegrees(m_PeriodicIO.targetPivotAngle));
    
            SignalLogger.writeDouble("Intake/CurrentDegrees", Math.toDegrees(m_PeriodicIO.currentPivotAngle));
            SignalLogger.writeDouble("Intake/TargetDegrees", Math.toDegrees(m_PeriodicIO.targetPivotAngle));
    }

}
