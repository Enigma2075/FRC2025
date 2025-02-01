package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemIO{

    private TalonFX m_pivot;
    private TalonFX m_roller;

    private final DutyCycleOut m_IntakeRequest= new DutyCycleOut(0);

    public Intake() {
        m_pivot = new TalonFX(IntakeConstants.kpivotId,RobotConstants.kCanivoreBusName);
        m_roller = new TalonFX(IntakeConstants.kroller,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        
        Slot0Configs slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kG = IntakeConstants.kG;
        slot0Configs.kS = IntakeConstants.kS;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;


        MotionMagicConfigs motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicAcceleration = 0;
        motionMagicConfigs.MotionMagicJerk = 0;
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
    State requestedState = State.CLIMBREADY;
    double enc = 0;

    double lastPosition = Double.MIN_VALUE;
}

private final PeriodicIO m_PeriodicIO = new PeriodicIO();

@Override
public void readPeriodicInputs() {
    m_PeriodicIO.enc = m_pivot.getPosition().getValueAsDouble();
    m_PeriodicIO.enc = m_roller.getPosition().getValueAsDouble();
}

private void writeIntake(double position) {
    if(m_PeriodicIO.lastPosition!= position) {
        m_pivot.setControl(m_IntakeRequest.withOutput(position));
        m_roller.setControl(m_IntakeRequest.withOutput(position));
        m_PeriodicIO.lastPosition = position;
    }
}

@Override
public void writePeriodicOutputs() {
    switch(m_PeriodicIO.requestedState)
    {
        case CLIMBREADY: 
            writeIntake(0);
        case FLOORINTAKE:
            writeIntake(0);
        case GRABCAGE:
            writeIntake(0);

    }
}

     @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

}
