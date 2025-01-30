package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Wrist extends SubsystemIO{
    public enum ControlMode{
        VOLTAGE,
        POSITION,
        SYSID
    }
    
    private TalonFX m_Motor;
    private CANcoder m_Encoder;

    private final VoltageOut m_VoltageRequest = new VoltageOut(0);

    public Wrist(){
        m_Motor = new TalonFX(WristConstants.kMotorId,RobotConstants.kCanivoreBusName);

        m_Encoder = new CANcoder(WristConstants.kEncoderId, RobotConstants.kCanivoreBusName);
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.VOLTAGE;
        public double CurrentAngle = 0;
        public double TargetAngle = 0;
    
        public double targetVoltage = 0;
        
    }

    public void setVoltage (double voltage) {
        m_PeriodicIO.controlMode = ControlMode.VOLTAGE;
        m_PeriodicIO.targetVoltage = voltage;
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

    @Override
    public void writePeriodicOutputs() {
           /*switch(m_PeriodicIO.controlMode){
                case VOLTAGE:
                    if(m_PeriodicIO.targetVoltage != m_PeriodicIO.lastTargetVoltage){
                        m_Motor.setControl(m_VoltageRequest.withOutput(m_PeriodicIO.targetVoltage));
                        m_PeriodicIO.lastTargetVoltage = m_PeriodicIO.targetVoltage;
                    }
                    break;
                case POSITION:
                    if(m_PeriodicIO.targetVoltage != m_PeriodicIO.lastTargetVoltage){
                        //m_Motor.setControl(m_VoltageRequest.withOutput(m_PeriodicIO.targetVoltage));
                        m_PeriodicIO.lastTargetVoltage = m_PeriodicIO.targetVoltage;
                    }
                    break;
                case SYSID:
                
                    break;
                default:

                    break;*/
            }
    }
    


