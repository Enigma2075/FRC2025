package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemIO{

    private TalonFX m_Back;
    private TalonFX m_Front;

    private final PositionVoltage m_ClimbRequest= new PositionVoltage(0);

    public Climb() {
        m_Back = new TalonFX(ClimbConstants.kBackId);
        m_Front = new TalonFX(ClimbConstants.kFrontId);
    }

    public enum State { 
        START(42), 
        ENDCLIMB(42);
        
        public final double distance;

        private State(double distance) {
            this.distance = distance;
        }
    }

    public static class PeriodicIO{
        State requestedState = State.START;
        double enc = 0;

        double lastPosition = Double.MIN_VALUE;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.enc = m_Back.getPosition().getValueAsDouble();
        m_PeriodicIO.enc = m_Front.getPosition().getValueAsDouble();
    }

    private void writeClimb(double position) {
        if(m_PeriodicIO.lastPosition!= position) {
            m_Back.setControl(m_ClimbRequest.withPosition(position));
            m_Front.setControl(m_ClimbRequest.withPosition(position));
            m_PeriodicIO.lastPosition = position;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.requestedState)
        {
            case START:
                    writeClimb(0);
            case ENDCLIMB:
                    writeClimb(0);
                    
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
