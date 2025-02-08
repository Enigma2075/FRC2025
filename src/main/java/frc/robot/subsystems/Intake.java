package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

    private final DutyCycleOut m_PivotOutputRequest= new DutyCycleOut(0);
    private final MotionMagicDutyCycle m_PivotPositionRequest = new MotionMagicDutyCycle(0).withSlot(0);
    private final DutyCycleOut m_RollerOutputRequest= new DutyCycleOut(0);


    public Intake() {
        m_pivot = new TalonFX(IntakeConstants.kPivotId,RobotConstants.kCanivoreBusName);
        m_roller = new TalonFX(IntakeConstants.kRollerId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_roller.getConfigurator().apply(rollerConfigs);

    }

    public enum PivotPosition { 
        GRABCAGE(9),
        FLOORINTAKE(9),
        CLIMBREADY(9);

        public final double angle;

        private PivotPosition(double angle) {
            this.angle = angle;
        }
    } 

    public static class PeriodicIO {

        public ControlMode controlMode = ControlMode.OUTPUT;    

        PivotPosition requestedPivotPosition = PivotPosition.CLIMBREADY;
        double pivotEncoder = 0;


        public double targetPivotOutput = 0;

        public double targetRollerOutput = 0;
    }

    public void setPivotOutput (double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetPivotOutput = output;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();


    public Command testCommand(Supplier<Double> outputPercent){
        return run(() -> {
            setPivotOutput(outputPercent.get());
        });
    }
    
    public Command setTestPosition(){
        return run(() -> {
            m_PeriodicIO.requestedPivotPosition = PivotPosition.GRABCAGE;
            m_PeriodicIO.controlMode = ControlMode.POSITION;

        });
    }

    public void setPivotPosition(PivotPosition position){
        m_PeriodicIO.requestedPivotPosition = position;
        m_PeriodicIO.controlMode = ControlMode.POSITION;
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.pivotEncoder = m_pivot.getPosition().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:   
                m_pivot.setControl(m_PivotOutputRequest.withOutput(m_PeriodicIO.targetPivotOutput));
                break;

            case POSITION:
                m_pivot.setControl(m_PivotPositionRequest.withPosition(m_PeriodicIO.requestedPivotPosition.angle));
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
       
    }

}
