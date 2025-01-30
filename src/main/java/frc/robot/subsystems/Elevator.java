package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemIO{

    private TalonFX m_ElevatorFront;
    private TalonFX m_ElevatorBack;

    public Elevator() {
        m_ElevatorFront = new TalonFX(ElevatorConst.kMotorFrontId,RobotConstants.kCanivoreBusName);
        m_ElevatorBack = new TalonFX(ElevatorConst.kMotorBackId,RobotConstants.kCanivoreBusName);
    }
   
    public static class PeriodicIO {
        public double targetHeight = ElevatorConst.kInitialHeight;

        public double currentHeight = 0;

        public double lastTargetHeight;
    
    }

    public void setHeight (double height){
        m_PeriodicIO.targetHeight = height;

    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToHeight(double position) {
        return position * ElevatorConst.kRotationToInches;
    }

    private double convertHeightToPosition(double height) {
        return height / ElevatorConst.kRotationToInches;
    }

    @Override
    public void stop() {
        
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Elevator/TargetHeight", m_PeriodicIO.targetHeight);
        SmartDashboard.putNumber("Elevator/CurrentHeight", m_PeriodicIO.currentHeight);

        SignalLogger.writeDouble("Elevator/TargetHeight", m_PeriodicIO.targetHeight);
        SignalLogger.writeDouble("Elevator/CurrentHeight", m_PeriodicIO.currentHeight);
    }
    
    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.currentHeight = convertPositionToHeight(m_ElevatorFront.getPosition().getValueAsDouble());
    }

    @Override
    public void writePeriodicOutputs() {
        if (m_PeriodicIO.lastTargetHeight != m_PeriodicIO.targetHeight) {
        //m_ElevatorFront.set(ControlMode.Position, convertHeightToPosition(m_PeriodicIO.targetHeight));
        //m_ElevatorBack.set(ControlMode.Follower, ElevatorConst.kMotorFrontId);
        m_PeriodicIO.lastTargetHeight = m_PeriodicIO.targetHeight;
 
        }

    }
}
