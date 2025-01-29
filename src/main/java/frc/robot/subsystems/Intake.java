package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemIO{

    private TalonFX m_pivot;
    private TalonFX m_roller;

    private final PositionVoltage m_IntakeRequest= new PositionVoltage(0);

    public Intake() {
        m_pivot = new TalonFX(IntakeConstants.kpivotId,RobotConstants.kCanivoreBusName);
        m_roller = new TalonFX(IntakeConstants.kroller,RobotConstants.kCanivoreBusName);
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
        m_pivot.setControl(m_IntakeRequest.withPosition(position));
        m_roller.setControl(m_IntakeRequest.withPosition(position));
        m_PeriodicIO.lastPosition = position;
    }
}

@Override
public void writePeriodicOutputs() {
    switch(m_PeriodicIO.requestedState)
    {
        case CLIMBREADY: 
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
