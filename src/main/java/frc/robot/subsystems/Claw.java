package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class Claw extends SubsystemIO {

    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_Coral;
    private TalonFX m_Algae;

    //private final Output m_Output = new Output(0);

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final MotionMagicDutyCycle m_PositionRequest = new MotionMagicDutyCycle(0).withSlot(0);
    
    public Claw() {
        m_Coral = new TalonFX(ClawConstants.kCoral,RobotConstants.kCanivoreBusName);
        m_Algae = new TalonFX(ClawConstants.kAlgae,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration coralConfigs = new TalonFXConfiguration();

        Slot0Configs slot0Configs = coralConfigs.Slot0;
        slot0Configs.kG = ClawConstants.kG;
        slot0Configs.kS = ClawConstants.kS;
        slot0Configs.kV = ClawConstants.kV;
        slot0Configs.kA = ClawConstants.kA;
        slot0Configs.kP = ClawConstants.kP;
        slot0Configs.kI = ClawConstants.kI;
        slot0Configs.kD = ClawConstants.kD;

        MotionMagicConfigs motionMagicConfigs = coralConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClawConstants.kMotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ClawConstants.kMotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClawConstants.kMotionMagicJerk;

        m_Coral.getConfigurator().apply(coralConfigs);
    }


    public enum State {
        L1(0),
        L2(0),
        L3(0),
        L4(0),
        INTAKE(0);

        public final double clawAngle;

        private State(double clawAngle) {
            this.clawAngle = clawAngle;
        }
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;
        State requestedState = State.INTAKE;

        double enc = 0;

        public double currentAngle = 0;
        public double targetAngle = 0;
        
        public double targetOutput=0;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setOutput(double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    private double convertPositionToAngle(double position) {
        return (position / ArmConstants.kGearRatio)* 2 * Math.PI ;
    }

    private double convertAngleToPosition(double angle) {
        return (angle / 2 * Math.PI) * ArmConstants.kGearRatio;
    }

    public Command testCommand(Supplier<Double> outputPercent) {
        return new Command() {
            @Override
            public void execute() {
                setOutput(outputPercent.get() * 12.0);
            }
        };
    }

    public Command setTestPosition(){
        return run(() -> {
            m_PeriodicIO.requestedState = State.INTAKE;
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

    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.enc = m_Coral.getPosition().getValueAsDouble();
        m_PeriodicIO.enc = m_Algae.getPosition().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:
                if(m_PeriodicIO.targetAngle != m_PeriodicIO.targetOutput) {
                    m_Algae.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                    m_PeriodicIO.targetOutput = m_PeriodicIO.targetOutput;
                }
                break;
            case POSITION:
                m_Algae.setControl(m_PositionRequest.withPosition(m_PeriodicIO.requestedState.clawAngle));
                break;
            
        }
    }
}
