package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemIO{
    
    private TalonFX m_Motor;

    

    public Wrist(){
        m_Motor = new TalonFX(WristConstants.kMotorId);
    }

    public static class PeriodicIO {
        public double CurrentAngle = 0;
        public double TargetAngle = 0;
    
        
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToAngle(double position) {
        return (position / WristConstants.kGearRatio)* 2 * Math.PI ;
    }

    private double convertAngleToPosition(double angle) {
        return (angle / 2 * Math.PI) * WristConstants.kGearRatio;
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
        SmartDashboard.putNumber("Wrist/CurrentAngle", m_PeriodicIO.CurrentAngle);
        SmartDashboard.putNumber("Wrist/TargetAngle", m_PeriodicIO.TargetAngle);

        SignalLogger.writeDouble("Wrist/CurrentAngle", m_PeriodicIO.CurrentAngle);
        SignalLogger.writeDouble("Wrist/TargetAngle", m_PeriodicIO.TargetAngle);
    }

    @Override
    public void readPeriodicInputs(){

    }

}
