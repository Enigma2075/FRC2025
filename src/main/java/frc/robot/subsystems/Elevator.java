package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.states.ElevatorState;

public class Elevator extends SubsystemIO{

    private TalonFX m_Elevator1;
    private TalonFX m_Elevator2;

    public Elevator() {
        m_Elevator1 = new TalonFX(ElevatorConst.kElevator1Id);
        m_Elevator2 = new TalonFX(ElevatorConst.kElevator2Id);
    }
    
   
    public static class PeriodicIO {
        
    }


    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

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
