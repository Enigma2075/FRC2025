package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemIO{
    public enum ControlMode {
        VOLTAGE,
        POSITION,
        SYSID
    }

    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private final Claw m_Claw;

    private TalonFX m_ElevatorFront;
    private TalonFX m_ElevatorBack;

    //private final MotionMagicVelocityTorqueCurrentFOC m_FlywheelRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    //private final MotionMagicVoltage m_PivotRequest = new MotionMagicVoltage(0);

    private final VoltageOut m_VoltageRequest = new VoltageOut(0);

    public Elevator(Arm arm, Wrist wrist, Claw claw) {
        m_Arm = arm;
        m_Wrist = wrist;
        m_Claw = claw;

        m_ElevatorFront = new TalonFX(ElevatorConst.kMotorFrontId,RobotConstants.kCanivoreBusName);
        m_ElevatorBack = new TalonFX(ElevatorConst.kMotorBackId,RobotConstants.kCanivoreBusName);
    }
   
    private static class PeriodicIO {
        public ControlMode controlMode = ControlMode.VOLTAGE;

        public double targetHeight = ElevatorConst.kInitialHeight;
        public double currentHeight = 0;
    
        public double targetVoltage = 0;
    }

    public void setVoltage (double voltage) {
        m_PeriodicIO.controlMode = ControlMode.VOLTAGE;
        m_PeriodicIO.targetVoltage = voltage;
    }

    public void setHeight (double height) {
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.targetHeight = height;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private double convertPositionToHeight(double position) {
        return position * ElevatorConst.kRotationToInches;
    }

    private double convertHeightToPosition(double height) {
        return height / ElevatorConst.kRotationToInches;
    }

    public Command testCommand(Supplier<Double> voltage) {
        return new Command() {
            @Override
            public void execute() {
                setVoltage(voltage.get());
            }
        };
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
        switch (m_PeriodicIO.controlMode) {
            case VOLTAGE:
                m_ElevatorFront.setControl(m_VoltageRequest.withOutput(m_PeriodicIO.targetVoltage));
                break;
            case POSITION:
                //m_ElevatorFront.setControl(m_FlywheelRequest.withOutput(m_PeriodicIO.targetHeight));
                break;
            case SYSID:

                break;
            default:
                break;
        }
    }
}
