package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Climb extends SubsystemIO{
     public enum ControlMode{
        //VOLTAGE,
        OUTPUT,
        POSITION,
        SYSID
     }

    private TalonFX m_Back;
    private TalonFX m_Front;
    private Servo m_Latch;

    //private final VoltageOut m_VoltageRequest = new VoltageOut(0);
    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final PositionVoltage m_ClimbRequest= new PositionVoltage(0);
    private final MotionMagicVoltage m_PositionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final StaticBrake m_StaticRequest = new StaticBrake();

     private final VoltageOut m_SysIdRequest = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volt.of(.8).per(Seconds),
                    Volts.of(2),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        m_Front.setControl(m_SysIdRequest.withOutput(m_SysIdRequest.withOutput(volts.in(Volts)).Output));
                        //System.out.println(volts.in(Volts));
                    },
                    (SysIdRoutineLog log) -> log.motor("Climb").voltage(m_Front.getMotorVoltage().getValue()).angularPosition(m_Front.getPosition().getValue()).angularVelocity(m_Front.getVelocity().getValue()),
                    (Subsystem)this));

    public Climb() {
        m_Back = new TalonFX(ClimbConstants.kBackId, RobotConstants.kCanivoreBusName);
        m_Front = new TalonFX(ClimbConstants.kFrontId, RobotConstants.kCanivoreBusName);
        m_Latch = new Servo(ClimbConstants.kLatchPort);

        TalonFXConfiguration frontConfig = new TalonFXConfiguration();
        frontConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        frontConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs slot0Configs = frontConfig.Slot0;
        slot0Configs.kG = ClimbConstants.kG;
        slot0Configs.kS = ClimbConstants.kS;
        slot0Configs.kV = ClimbConstants.kV;
        slot0Configs.kA = ClimbConstants.kA;
        slot0Configs.kP = ClimbConstants.kP;
        slot0Configs.kI = ClimbConstants.kI;
        slot0Configs.kD = ClimbConstants.kD;

        MotionMagicConfigs motionMagicConfigs = frontConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.kMotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.kMotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClimbConstants.kMotionMagicJerk;
        
        m_Front.getConfigurator().apply(frontConfig);

        TalonFXConfiguration backConfig = new TalonFXConfiguration();

        backConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_Back.getConfigurator().apply(backConfig);

        m_Back.setControl(new Follower(m_Front.getDeviceID(), true));

        m_Front.setPosition(0);
    }

    public enum State { 
        START(0), 
        ENDCLIMB(130);
        
        public final double distance;

        private State(double distance) {
            this.distance = distance;
        }
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;

        State requestedState = State.START;

        public double currentPosition = 0;
        
        //public double targetVoltage = 0;
        public double targetOutput = 0;
    }

    private double timeout = 0;

    /* 
    public void setVoltage (double voltage) {
        m_PeriodicIO.controlMode = ControlMode.VOLTAGE;
        m_PeriodicIO.targetVoltage = voltage;
    }
    */

    public Command setServo() {
        return runOnce(() -> {
            m_Latch.set(1);
            timeout = Timer.getFPGATimestamp();
        });
    }

    public void setOutput (double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public Command testCommand(Supplier<Double> outputPercent){
        return run(() -> {
            setOutput(outputPercent.get());
        });
    }

    public Command setTestPosition(){
        return run(() -> {
            m_PeriodicIO.requestedState = State.START;
            m_PeriodicIO.controlMode = ControlMode.POSITION;

        });
    }

    public Command moveToPosition(State state){
        return run(() -> { 
        m_PeriodicIO.requestedState = state;
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        });
    }

    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.quasistatic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.dynamic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.currentPosition = m_Front.getPosition().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT :
                m_Front.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                break;

            case POSITION:
                m_Front.setControl(m_PositionRequest.withPosition(m_PeriodicIO.requestedState.distance));
                break;

            case SYSID:

                break;

            default:

                break;
        }
        if(Timer.getFPGATimestamp()-timeout>1){
            m_Latch.setDisabled();
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
        SmartDashboard.putNumber("Climb/CurrentPosition", m_PeriodicIO.currentPosition);
        
        SignalLogger.writeDouble("Climb/CurrentPosition", m_PeriodicIO.currentPosition);
    }

}
