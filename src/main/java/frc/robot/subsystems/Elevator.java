package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemIO{

    private TalonFX m_Elevator1;
    private TalonFX m_Elevator2;

    public Elevator() {
        m_Elevator1 = new TalonFX(ElevatorConst.kElevator1Id);
        m_Elevator2 = new TalonFX(ElevatorConst.kElevator2Id);
    }

    public enum State {
        L4, L3, L2, L1,
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
