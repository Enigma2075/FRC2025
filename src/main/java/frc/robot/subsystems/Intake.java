package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemIO{
    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_pivot;
    private TalonFX m_roller;

    private final DutyCycleOut m_OutputRequest= new DutyCycleOut(0);
    private final PositionVoltage m_IntakeRequest = new PositionVoltage(0);
    private final MotionMagicDutyCycle m_PositionRequest = new MotionMagicDutyCycle(0).withSlot(0);
    

    public Intake() {
        m_pivot = new TalonFX(IntakeConstants.kPivotId,RobotConstants.kCanivoreBusName);
        m_roller = new TalonFX(IntakeConstants.kRollerId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

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
    }

    public enum State { 
        GRABCAGE(9),
        FLOORINTAKE(9),
        CLIMBREADY(9);

        public final double angle;

        private State(double angle) {
            this.angle = angle;
        }
    } 

    public static class PeriodicIO {

        public ControlMode controlMode = ControlMode.OUTPUT;    

        State requestedState = State.CLIMBREADY;
        double enc = 0;

        double lastPosition = Double.MIN_VALUE;

        public double targetOutput = 0;
    }

    public void setOutput (double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private void writeIntake(double position) {
        if(m_PeriodicIO.lastPosition!= position) {
            m_pivot.setControl(m_OutputRequest.withOutput(position));
            m_roller.setControl(m_OutputRequest.withOutput(position));
            m_PeriodicIO.lastPosition = position;
        }
    }

    public Command setTestPosition(){
        return run(() -> {
            m_PeriodicIO.requestedState = State.GRABCAGE;
            m_PeriodicIO.controlMode = ControlMode.POSITION;

        });
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.enc = m_pivot.getPosition().getValueAsDouble();
        m_PeriodicIO.enc = m_roller.getPosition().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:
                
                break;

            case POSITION:
                m_pivot.setControl(m_PositionRequest.withPosition(m_PeriodicIO.requestedState.angle));
                break;

            case SYSID:

                break;

            default:

                break;
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
       
    }

}
