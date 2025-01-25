package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemIO{

    private TalonFX m_pivot;
    private TalonFX m_roller;

    public Intake() {
        m_pivot = new TalonFX(IntakeConstants.kpivotId);
        m_roller = new TalonFX(IntakeConstants.kroller);
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
