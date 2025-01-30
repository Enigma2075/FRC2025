package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemIO{

    public enum ControlMode{
        VOLTAGE,
        POSITION,
        SYSID
    }

    private TalonFX m_Motor;
    private CANcoder m_Encoder;

    private final VoltageOut m_VoltageRequest = new VoltageOut(0);


    public Arm(){
        m_Motor = new TalonFX(ArmConstants.kMotorId, RobotConstants.kCanivoreBusName);
        m_Encoder = new CANcoder(ArmConstants.kEncoderId, RobotConstants.kCanivoreBusName);
    }

    public static class PeriodicIO {
        public ControlMode controlMode = ControlMode.VOLTAGE;

        public double currentAngle = 0;
        public double targetAngle = 0;

        public double targetVoltage = 0; 
        
    }

    public void setVoltage(double voltage){
        m_PeriodicIO.controlMode = ControlMode.VOLTAGE;
        m_PeriodicIO.targetVoltage = voltage;
    }

    public void setAngle(double angle){
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.targetAngle = angle;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToAngle(double position) {
        return (position / ArmConstants.kGearRatio)* 2 * Math.PI ;
    }

    private double convertAngleToPosition(double angle) {
        return (angle / 2 * Math.PI) * ArmConstants.kGearRatio;
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
        SmartDashboard.putNumber("Arm/CurrentAngle", m_PeriodicIO.currentAngle);
        SmartDashboard.putNumber("Arm/TargetAngle", m_PeriodicIO.targetAngle);

        SignalLogger.writeDouble("Arm/CurrentAngle", m_PeriodicIO.currentAngle);
        SignalLogger.writeDouble("Arm/TargetAngle", m_PeriodicIO.targetAngle);
    }
    
    @Override
    public void readPeriodicInputs(){
        
    }

    @Override
    public void writePeriodicOutputs(){
        switch(m_PeriodicIO.controlMode){
            case VOLTAGE:
                
                m_Motor.setControl(m_VoltageRequest.withOutput(m_PeriodicIO.targetVoltage));
                   
                break;
            case POSITION:
                
                break;
            case SYSID:

                break;
            default:

                break;
        }
    }
}
