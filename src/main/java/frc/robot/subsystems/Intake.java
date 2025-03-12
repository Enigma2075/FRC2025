package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
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
import frc.robot.RobotState;

public class Intake extends SubsystemIO{
    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_pivot;
    private TalonFX m_roller;

    private CANrange m_Sensor;

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
        m_Sensor = new CANrange(IntakeConstants.kSensorId, RobotConstants.kCanivoreBusName);

        CANrangeConfiguration sensorConfig = new CANrangeConfiguration();

        sensorConfig.FovParams.FOVCenterX = 6;
        sensorConfig.FovParams.FOVCenterY = 11;
        sensorConfig.FovParams.FOVRangeX = 7;
        sensorConfig.FovParams.FOVRangeY = 7;

        m_Sensor.getConfigurator().apply(sensorConfig);


        m_pivot = new TalonFX(IntakeConstants.kPivotId,RobotConstants.kCanivoreBusName);
        m_roller = new TalonFX(IntakeConstants.kRollerId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        Slot0Configs slot0Configs = pivotConfigs.Slot0;
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

        setState(States.DEFAULT);
    }

    public enum States { 
        GRABCAGE(95, 0),
        FLOORINTAKE(53, 1),
        OUTTAKE(110, -1),
        CLIMBREADY(0, 0),
        DEFAULT(110, 0),
        DISABLE(0, 0),
        HANDOFFALGAE(100, 0);

        public final double degrees;
        public final double rollerOutput;

        private States(double degrees, double rollerOutput) {
            this.degrees = degrees;
            this.rollerOutput = rollerOutput;
        }
    } 

    public static class PeriodicIO {
        public ControlMode controlMode = ControlMode.OUTPUT;    

        States targetPivotPosition = States.DEFAULT;
        
        public double targetPivotOutput = 0;
        
        public double targetPivotAngle = 0;
        public double currentPivotAngle = 0;

        public double targetRollerOutput = 0;
        public double currentRollerCurrent = 0;

        public double currentDistance = 0;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setPivotOutput (double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetPivotOutput = output;
    }

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

    //set a command that will return a command that will set the output 
    //public Command setOutputCommand(double output){
    //    return run(()->{
    //        setPivotOutput(output);
    //    });
    //}

    //return a command that will set the position
    public Command setStateCommand(States state){
        return runOnce(()->{
            if(state == States.CLIMBREADY) {
                RobotState.isClimbing = true;
            }
            setState(state);
        });
    }
    
    public Command setTestPosition(){
        return run(() -> {
            m_PeriodicIO.targetPivotPosition = States.GRABCAGE;
            m_PeriodicIO.controlMode = ControlMode.POSITION;
        });
    }

    public void setState(States position){
        m_PeriodicIO.targetPivotPosition = position;
        m_PeriodicIO.targetPivotAngle = Math.toRadians(position.degrees);
        m_PeriodicIO.targetRollerOutput = position.rollerOutput;
        m_PeriodicIO.controlMode = ControlMode.POSITION;
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.currentPivotAngle = convertPositionToAngle(m_pivot.getPosition().getValueAsDouble());
        m_PeriodicIO.currentRollerCurrent = m_roller.getSupplyCurrent().getValueAsDouble();
        m_PeriodicIO.currentDistance = m_Sensor.getDistance().getValue().in(Centimeters);
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:   
                m_pivot.setControl(m_PivotOutputRequest.withOutput(m_PeriodicIO.targetPivotOutput));
                break;
            case POSITION:
                if(m_PeriodicIO.targetPivotPosition == States.DISABLE) {
                    m_pivot.disable();
                }
                else {
                    m_pivot.setControl(m_PivotPositionRequest.withPosition(convertAngleToPosition(m_PeriodicIO.targetPivotAngle)).withFeedForward(Math.cos(m_PeriodicIO.currentPivotAngle) * IntakeConstants.kG));
                }
                break;
            case SYSID:

                break;
            default:

                break;
        }

        if(m_PeriodicIO.targetRollerOutput == 0 && m_PeriodicIO.currentDistance < IntakeConstants.kMaxRange && m_PeriodicIO.currentDistance > IntakeConstants.kMinRange) {
            m_roller.setControl(m_RollerOutputRequest.withOutput(.1));
        }
        else {
            m_roller.setControl(m_RollerOutputRequest.withOutput(m_PeriodicIO.targetRollerOutput));
        }
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
            SmartDashboard.putNumber("Intake/CurrentDistance", m_PeriodicIO.currentDistance);
    
            SignalLogger.writeDouble("Intake/CurrentDegrees", Math.toDegrees(m_PeriodicIO.currentPivotAngle));
            SignalLogger.writeDouble("Intake/TargetDegrees", Math.toDegrees(m_PeriodicIO.targetPivotAngle));
            SignalLogger.writeDouble("Intake/CurrentDistance", m_PeriodicIO.currentDistance);
    }

}
