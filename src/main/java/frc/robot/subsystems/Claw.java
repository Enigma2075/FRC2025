package frc.robot.subsystems;


import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.function.BiPredicate;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class Claw extends SubsystemIO {
    public enum AlgaeModes { 
        STOP(0,0, null, null),
        HOLD(20, 16, STOP, (Double actualCurrent, Double thresholdCurrent) -> actualCurrent < thresholdCurrent),
        INTAKE(40, 30, HOLD, (Double actualCurrent, Double thresholdCurrent) -> actualCurrent > thresholdCurrent),
        OUTTAKE(-40, -18, STOP, (Double actualCurrent, Double thresholdCurrent) -> actualCurrent > thresholdCurrent);

        public final double current;
        private final double thresholdCurrent;
        public final AlgaeModes next;
        private final BiPredicate<Double, Double> checkThreshold;

        private AlgaeModes(double current, double thresholdCurrent, AlgaeModes next, BiPredicate<Double, Double> checkTimeout) {
            this.current = current;
            this.thresholdCurrent = thresholdCurrent;
            this.next = next;
            this.checkThreshold = checkTimeout;
        }

        public boolean isOverThreshold(double actualCurrent) {
            return checkThreshold.test(actualCurrent, thresholdCurrent);
        }
    } 

    public enum CoralModes { 
        STOP(0,0, null, null),
        HOLD(20, 16, STOP, (Double actualCurrent, Double thresholdCurrent) -> actualCurrent < thresholdCurrent),
        INTAKE(40, 25, HOLD, (Double actualCurrent, Double thresholdCurrent) -> actualCurrent > thresholdCurrent),
        OUTTAKE(-40, -18, STOP, (Double actualCurrent, Double thresholdCurrent) -> actualCurrent > thresholdCurrent);

        public final double current;
        private final double thresholdCurrent;
        public final CoralModes next;
        private final BiPredicate<Double, Double> checkThreshold;

        private CoralModes(double current, double thresholdCurrent, CoralModes next, BiPredicate<Double, Double> checkTimeout) {
            this.current = current;
            this.thresholdCurrent = thresholdCurrent;
            this.next = next;
            this.checkThreshold = checkTimeout;
        }

        public boolean isOverThreshold(double actualCurrent) {
            return checkThreshold.test(actualCurrent, thresholdCurrent);
        }
    } 

    private TalonFX m_Coral;
    private TalonFX m_Algae;
    
    private CANrange m_CoralSensor;
    private CANrange m_AlgaeSensor;

    private final TorqueCurrentFOC m_OutputRequest = new TorqueCurrentFOC(0);
    
    public Claw() {
        m_CoralSensor = new CANrange(ClawConstants.kCoralSensorId, RobotConstants.kCanivoreBusName);

        CANrangeConfiguration coralSensorConfig = new CANrangeConfiguration();

        coralSensorConfig.FovParams.FOVCenterX = 1;
        coralSensorConfig.FovParams.FOVCenterY = 1;
        coralSensorConfig.FovParams.FOVRangeX = 8;
        coralSensorConfig.FovParams.FOVRangeY = 8;

        m_CoralSensor.getConfigurator().apply(coralSensorConfig);

        m_AlgaeSensor = new CANrange(ClawConstants.kAlgaeSensorId, RobotConstants.kCanivoreBusName);

        CANrangeConfiguration algaeSensorConfig = new CANrangeConfiguration();

        algaeSensorConfig.FovParams.FOVCenterX = 1;
        algaeSensorConfig.FovParams.FOVCenterY = 1;
        algaeSensorConfig.FovParams.FOVRangeX = 8;
        algaeSensorConfig.FovParams.FOVRangeY = 8;

        m_AlgaeSensor.getConfigurator().apply(algaeSensorConfig);


        m_Algae = new TalonFX(ClawConstants.kAlgaeId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration algaeConfigs = new TalonFXConfiguration();

        algaeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algaeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_Algae.getConfigurator().apply(algaeConfigs);

        m_Coral = new TalonFX(ClawConstants.kCoralId,RobotConstants.kCanivoreBusName);

        TalonFXConfiguration coralConfigs = new TalonFXConfiguration();

        coralConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        coralConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_Coral.getConfigurator().apply(coralConfigs);

        setAlgaeMode(AlgaeModes.HOLD);
        setCoralMode(CoralModes.HOLD);
    }

    public static class PeriodicIO{
        public CoralModes coralMode = CoralModes.STOP;

        public double firstSeenCoral = Double.MIN_VALUE;
        public boolean hasCoral = false;
        public boolean latchCoralIntake = false;
        
        public AlgaeModes algaeMode = AlgaeModes.STOP;

        public double firstSeenAlgae = Double.MIN_VALUE;
        public boolean hasAlgae = false;
        public boolean latchAlgaeIntake = false;
   }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setCoralMode(CoralModes mode){
            m_PeriodicIO.coralMode = mode;
            if(mode == CoralModes.INTAKE) {
                m_PeriodicIO.latchCoralIntake = true;
            }

            if(mode == CoralModes.INTAKE && m_PeriodicIO.hasCoral && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenCoral > 1.5) {
                m_PeriodicIO.firstSeenCoral = Double.MIN_VALUE;
            }
    }

    public void setAlgaeMode(AlgaeModes mode){
        setAlgaeMode(mode, false);
    }

    public void setAlgaeMode(AlgaeModes mode, boolean disableTimeout){
        m_PeriodicIO.algaeMode = mode;
        if(mode == AlgaeModes.INTAKE) {
            m_PeriodicIO.latchAlgaeIntake = true;
        }

        if(mode == AlgaeModes.INTAKE && m_PeriodicIO.hasAlgae && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenAlgae > 1.5) {
            m_PeriodicIO.firstSeenAlgae = Double.MIN_VALUE;
        }
    }

    public boolean hasAlgae() {
        return m_PeriodicIO.algaeMode == AlgaeModes.HOLD;
    }

    public boolean hasCoral() {
        return m_PeriodicIO.hasCoral;
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
        //TODO:SmartDashboard.putNumber("Claw/AlgaeCurrent", m_PeriodicIO.algaeCurrent);
        SmartDashboard.putBoolean("Claw/HasCoral", m_PeriodicIO.hasCoral);
        SmartDashboard.putString("Claw/AlgaeMode", m_PeriodicIO.algaeMode.name());
        SmartDashboard.putString("Claw/CoralMode", m_PeriodicIO.coralMode.name());

        //TODO:SignalLogger.writeDouble("Claw/AlgaeCurrent", m_PeriodicIO.algaeCurrent);
        SignalLogger.writeBoolean("Claw/HasCoral", m_PeriodicIO.hasCoral);
        SignalLogger.writeString("Claw/AlgaeMode", m_PeriodicIO.algaeMode.name());
        SignalLogger.writeString("Claw/CoralMode", m_PeriodicIO.coralMode.name());
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.hasCoral = m_CoralSensor.getDistance().getValue().in(Centimeters) < 10 && m_CoralSensor.getIsDetected().getValue();
        if(m_PeriodicIO.hasCoral && m_PeriodicIO.firstSeenCoral == Double.MIN_VALUE) {
            m_PeriodicIO.firstSeenCoral = Timer.getFPGATimestamp();
        }
        else if(!m_PeriodicIO.hasCoral && m_PeriodicIO.firstSeenCoral != Double.MIN_VALUE) {
            m_PeriodicIO.firstSeenCoral = Timer.getFPGATimestamp();
        }

        //TODO:m_PeriodicIO.algaeCurrent = m_Algae.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        if(m_PeriodicIO.algaeMode != AlgaeModes.STOP) {
            //TODO:if(m_PeriodicIO.algaeMode.isOverThreshold(m_PeriodicIO.algaeCurrent)){ 
                //TODO:m_PeriodicIO.algaeTimeoutCount++;
            //TODO:}

            //TODO:if(m_PeriodicIO.algaeTimeoutCount > 7) {
            //TODO:    setAlgaeMode(m_PeriodicIO.algaeMode.next);
            //TODO:}
            //TODO:else if(m_PeriodicIO.algaeLastCheck - Timer.getFPGATimestamp() >= .02 * 12){
            //TODO:    m_PeriodicIO.algaeLastCheck = Timer.getFPGATimestamp();
             //TODO:   m_PeriodicIO.algaeTimeoutCount = 0;
            //TODO:}
            m_Algae.setControl(m_OutputRequest.withOutput(m_PeriodicIO.algaeMode.current));
        }
        else {
            m_Algae.stopMotor();
        }

        if(m_PeriodicIO.coralMode != CoralModes.STOP) {
            if(m_PeriodicIO.coralMode == CoralModes.INTAKE) {
                if(hasCoral() && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenCoral > .5) {
                    m_PeriodicIO.coralMode = CoralModes.HOLD;
                }
                else if(m_PeriodicIO.latchCoralIntake) {
                    m_PeriodicIO.latchCoralIntake = false;
                }
                else {
                    m_PeriodicIO.coralMode = CoralModes.STOP;
                }
            }
            else if(!hasCoral() && m_PeriodicIO.coralMode == CoralModes.HOLD) {
                m_PeriodicIO.coralMode = CoralModes.STOP;
            }
            else if(!hasCoral() && m_PeriodicIO.coralMode == CoralModes.OUTTAKE) {
                m_PeriodicIO.coralMode = CoralModes.STOP;
            }

            m_Coral.setControl(m_OutputRequest.withOutput(m_PeriodicIO.coralMode.current));
        }
        else {
            m_Coral.stopMotor();
        }
    }
}
