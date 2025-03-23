package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemIO{
    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_Front;
    private TalonFX m_Back;

    //private final MotionMagicVelocityTorqueCurrentFOC m_FlywheelRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    //private final MotionMagicVoltage m_PivotRequest = new MotionMagicVoltage(0);

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    private final DynamicMotionMagicVoltage m_PositionRequest = new DynamicMotionMagicVoltage(0, ElevatorConst.kMotionMagicCruiseVelocity, ElevatorConst.kMotionMagicAcceleration, ElevatorConst.kMotionMagicJerk).withSlot(0);
    
    private final VoltageOut m_SysIdRequest = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Volts.of(3).per(Seconds.of(1)),
                    null, // Volts.of(4.5),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        m_Front.setControl(m_SysIdRequest.withOutput(m_SysIdRequest.withOutput(volts.in(Volts)).Output));
                        //System.out.println(volts.in(Volts));
                    },
                    (SysIdRoutineLog log) -> log.motor("Elevator").voltage(m_Front.getMotorVoltage().getValue()).angularPosition(m_Front.getPosition().getValue()).angularVelocity(m_Front.getVelocity().getValue()),
                    (Subsystem)this));
                

    public Elevator() {
        m_Front = new TalonFX(ElevatorConst.kMotorFrontId,RobotConstants.kCanivoreBusName);
        m_Back = new TalonFX(ElevatorConst.kMotorBackId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration backConfig = new TalonFXConfiguration();

        backConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        backConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerLimit(.75)
            .withSupplyCurrentLimitEnable(true)
        );

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

        frontConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerLimit(.75)
            .withSupplyCurrentLimitEnable(true)
        );

        m_Front.getConfigurator().apply(frontConfig);

        m_Front.setControl(m_OutputRequest.withOutput(0));
        
    }
   
    private static class PeriodicIO {
        public ControlMode controlMode = ControlMode.OUTPUT;

        double enc = 0;

        double lastPosition = Double.MIN_VALUE;

        boolean overrideVelocity = false;

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

    public double getHeight(){
        return m_PeriodicIO.currentHeight;
    }

    public double getMaxHeight() {
        return ElevatorConst.kMaxHeight;
    }

    public double getMaxHeightWithoutOffset() {
        return ElevatorConst.kMaxHeight - ElevatorConst.kHeightOffset;
    }

    public double getHeightWithoutOffset() {
        return m_PeriodicIO.currentHeight - ElevatorConst.kHeightOffset;
    }

    private double convertPositionToHeight(double position) {
        double rawHeight = ElevatorConst.kHeightOffset + (position * ElevatorConst.kRotationToInches);
        return rawHeight + (rawHeight * ElevatorConst.kErrorCorrectionRatio);
    }

    private double convertHeightToPosition(double height) {
        double errorCorrection = height * ElevatorConst.kErrorCorrectionRatio;
        double position = (height - 7 - errorCorrection) / ElevatorConst.kRotationToInches;
        SmartDashboard.putNumber("Elevator/TargetPosition", position);

        return position;
    }

    public boolean isAtPosition(double height) {
        if(Math.abs(height - m_PeriodicIO.currentHeight) < 5) {
            return true;
        }
        else {
            return false;
        }        
    }

    public boolean isAtPosition() {
        return isAtPosition(m_PeriodicIO.targetHeight);
    }

    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.quasistatic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(() -> m_PeriodicIO.controlMode = ControlMode.SYSID).andThen(m_SysIdRoutine.dynamic(direction)).finallyDo(() -> m_PeriodicIO.controlMode = ControlMode.OUTPUT);
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

    public void setOverrideVelocity(boolean override) {
        m_PeriodicIO.overrideVelocity = override;
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
                if(m_PeriodicIO.overrideVelocity) {
                    m_Front.setControl(m_PositionRequest.withPosition(convertHeightToPosition(m_PeriodicIO.targetHeight)).withVelocity(ElevatorConst.kMotionMagicCruiseVelocity).withAcceleration(ElevatorConst.kMotionMagicAcceleration * .5));
                }
                else {
                    m_Front.setControl(m_PositionRequest.withPosition(convertHeightToPosition(m_PeriodicIO.targetHeight)));
                }
                break;
            case SYSID:

                break;
            default:
                break;
        }
    }
}
