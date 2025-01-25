package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemIO{

    private TalonFX m_Back;
    private TalonFX m_Front;

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
