package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.states.ElevatorState;

public class Elevator extends SubsystemIO{

    private TalonFX m_ElevatorFront;
    private TalonFX m_ElevatorBack;

    public Elevator() {
        m_ElevatorFront = new TalonFX(ElevatorConst.kMotorFrontId);
        m_ElevatorBack = new TalonFX(ElevatorConst.kMotorBackId);
    }
<<<<<<< HEAD


=======
    
   
>>>>>>> 5a567188011b45760d8e35e2e5c7535a4f4ce0f4
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
