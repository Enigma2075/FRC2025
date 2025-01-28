package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Arm extends SubsystemIO{

    private TalonFX m_Motor;

    public Arm(){
        m_Motor = new TalonFX(ArmConstants.kMotorId);
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
