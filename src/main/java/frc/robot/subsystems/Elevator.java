package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemIO{
    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private final Claw m_Claw;

    private TalonFX m_Front;
    private TalonFX m_Back;

    //private final MotionMagicVelocityTorqueCurrentFOC m_FlywheelRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    //private final MotionMagicVoltage m_PivotRequest = new MotionMagicVoltage(0);

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final MotionMagicDutyCycle m_PositionRequest = new MotionMagicDutyCycle(0).withSlot(0);
    

    public Elevator(Arm arm, Wrist wrist, Claw claw) {
        m_Arm = arm;
        m_Wrist = wrist;
        m_Claw = claw;

        m_Front = new TalonFX(ElevatorConst.kMotorFrontId,RobotConstants.kCanivoreBusName);
        m_Back = new TalonFX(ElevatorConst.kMotorBackId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration backConfig = new TalonFXConfiguration();

        backConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_Back.getConfigurator().apply(backConfig);
        
        m_Back.setControl(new Follower(m_Front.getDeviceID(), true));

        TalonFXConfiguration frontConfig = new TalonFXConfiguration();

        frontConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        frontConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs slot0Configs = frontConfig.Slot0;
        slot0Configs.kG = ElevatorConst.kG;
        slot0Configs.kS = ElevatorConst.kS;
        slot0Configs.kV = ElevatorConst.kV;
        slot0Configs.kA = ElevatorConst.kA;
        slot0Configs.kP = ElevatorConst.kP;
        slot0Configs.kI = ElevatorConst.kI;
        slot0Configs.kD = ElevatorConst.kD;

        MotionMagicConfigs motionMagicConfigs = frontConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConst.kMotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConst.kMotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ElevatorConst.kMotionMagicJerk;

        m_Front.getConfigurator().apply(frontConfig);

    }

    
   
    private static class PeriodicIO {
        public ControlMode controlMode = ControlMode.OUTPUT;

        double enc = 0;

        double lastPosition = Double.MIN_VALUE;


        public double targetHeight = ElevatorConst.kInitialHeight;
        public double currentHeight = 0;
    
        public double targetOutput = 0;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setOutput (double output) {
        m_PeriodicIO.controlMode = ControlMode.OUTPUT;
        m_PeriodicIO.targetOutput = output;
    }

    public void setHeight (double height) {
        m_PeriodicIO.controlMode = ControlMode.POSITION;
        m_PeriodicIO.targetHeight = height;
    }

    private double convertPositionToHeight(double position) {
        return position * ElevatorConst.kRotationToInches;
    }

    private double convertHeightToPosition(double height) {
        return height / ElevatorConst.kRotationToInches;
    }

    public Command testCommand(Supplier<Double> outputPercent) {
        return run(() -> {
                setOutput(outputPercent.get());
            });
    }

    public Command setTestPosition(double height){
        return run(()-> {
                setHeight(height);
        });
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
        m_PeriodicIO.currentHeight = convertPositionToHeight(m_Front.getPosition().getValueAsDouble());
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:
                m_Front.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                break;
            case POSITION:
                //m_ElevatorFront.setControl(m_FlywheelRequest.withOutput(m_PeriodicIO.targetHeight));
                m_Front.setControl(m_PositionRequest.withPosition(m_PeriodicIO.targetHeight));
                break;
            case SYSID:

                break;
            default:
                break;
        }
    }
}
