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

public class Arm extends SubsystemIO{

    public enum ControlMode{
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_Motor;
    private CANcoder m_Encoder;

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);


    public Arm(){
        m_Motor = new TalonFX(ArmConstants.kMotorId, RobotConstants.kCanivoreBusName);
        m_Encoder = new CANcoder(ArmConstants.kEncoderId, RobotConstants.kCanivoreBusName);
    }

    public static class PeriodicIO {
        public ControlMode controlMode = ControlMode.OUTPUT;

        public double currentAngle = 0;
        public double targetAngle = 0;

        public double targetOutput = 0; 
        
    }

    public void setOutput(double output){
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
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
            case OUTPUT:
                
                m_Motor.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                   
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
