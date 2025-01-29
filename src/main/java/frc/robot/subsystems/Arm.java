package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemIO{

    private TalonFX m_Motor;


    public Arm(){
        m_Motor = new TalonFX(ArmConstants.kMotorId);
    }

    public static class PeriodicIO {
        public double currentAngle = 0;
        public double targetAngle = 0;
    
        
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
        SmartDashboard.putNumber("Arm/CurrentAngle", m_PeriodicIO.currentAngle);
        SmartDashboard.putNumber("Arm/TargetAngle", m_PeriodicIO.targetAngle);

        SignalLogger.writeDouble("Arm/CurrentAngle", m_PeriodicIO.currentAngle);
        SignalLogger.writeDouble("Arm/TargetAngle", m_PeriodicIO.targetAngle);
    }
    
    @Override
    public void readPeriodicInputs(){
        
    }
}
