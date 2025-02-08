package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends SubsystemIO{
     public enum ControlMode{
        //VOLTAGE,
        OUTPUT,
        POSITION,
        SYSID
     }

    private TalonFX m_Back;
    private TalonFX m_Front;

    //private final VoltageOut m_VoltageRequest = new VoltageOut(0);
    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final PositionVoltage m_ClimbRequest= new PositionVoltage(0);
    private final MotionMagicDutyCycle m_PositionRequest = new MotionMagicDutyCycle(0).withSlot(0);
    private final StaticBrake m_StaticRequest = new StaticBrake();

    public Climb() {
        m_Back = new TalonFX(ClimbConstants.kBackId, RobotConstants.kCanivoreBusName);
        m_Front = new TalonFX(ClimbConstants.kFrontId, RobotConstants.kCanivoreBusName);

        TalonFXConfiguration frontConfig = new TalonFXConfiguration();
        frontConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        frontConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        frontConfig.MotorOutput.DutyCycleNeutralDeadband = .05;

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

        m_Back.getConfigurator().apply(backConfig);

        m_Back.setControl(new Follower(m_Front.getDeviceID(), true));
    }

    public enum State { 
        START(10), 
        ENDCLIMB(42);
        
        public final double distance;

        private State(double distance) {
            this.distance = distance;
        }
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;

        State requestedState = State.START;
        double enc = 0;

        double lastPosition = Double.MIN_VALUE;

        //public double targetVoltage = 0;
        public double targetOutput = 0;
    }

    /* 
    public void setVoltage (double voltage) {
        m_PeriodicIO.controlMode = ControlMode.VOLTAGE;
        m_PeriodicIO.targetVoltage = voltage;
    }
    */

    

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

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.enc = m_Back.getPosition().getValueAsDouble();
        //m_PeriodicIO.enc = m_Front.getPosition().getValueAsDouble();
    }

    private void writeClimb(double position) {
        if(m_PeriodicIO.lastPosition!= position) {
            m_Back.setControl(m_ClimbRequest.withPosition(position));
            //m_Front.setControl(m_ClimbRequest.withPosition(position));
            m_PeriodicIO.lastPosition = position;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT :
                
                m_Back.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                 
                break;

            case POSITION:
                m_Back.setControl(m_PositionRequest.withPosition(m_PeriodicIO.requestedState.distance));
                break;

            case SYSID:

                break;

            default:

                break;
        }

    }

        /*switch (m_PeriodicIO.requestedState)
        {
            case START:
                    writeClimb(0);
            case ENDCLIMB:
                    writeClimb(0);
                    
        }*/
    

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
