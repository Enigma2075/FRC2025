package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Wrist extends SubsystemIO{
    public enum ControlMode{
        OUTPUT,
        POSITION,
        SYSID
    }
    
    private TalonFX m_Motor;
    private CANcoder m_Encoder;

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);

    public Wrist(){
        m_Motor = new TalonFX(WristConstants.kMotorId,RobotConstants.kCanivoreBusName);

        m_Encoder = new CANcoder(WristConstants.kEncoderId, RobotConstants.kCanivoreBusName);
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;
        public double CurrentAngle = 0;
        public double TargetAngle = 0;
    
        public double targetOutput = 0;
        
    }

    public void setOutput (double output) {
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    public void setAngle(double angle){
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.TargetAngle = angle;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToAngle(double position) {
        return (position / WristConstants.kGearRatio)* 2 * Math.PI ;
    }

    private double convertAngleToPosition(double angle) {
        return (angle / 2 * Math.PI) * WristConstants.kGearRatio;
    }

    public Command testCommand(Supplier<Double> outputPercent) {
        return new Command() {
            @Override
            public void execute() {
                setOutput(outputPercent.get() * 12.0);
            }
        };
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
    


