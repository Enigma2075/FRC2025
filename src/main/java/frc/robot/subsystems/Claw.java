package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class Claw extends SubsystemIO {
    private TalonFXS m_Coral;
    private TalonFXS m_Algae;

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);
    
    public Claw() {
        m_Algae = new TalonFXS(ClawConstants.kAlgae,RobotConstants.kCanivoreBusName);

        TalonFXSConfiguration algaeConfigs = new TalonFXSConfiguration();

        algaeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algaeConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        algaeConfigs.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;

        algaeConfigs.CurrentLimits.StatorCurrentLimit = 40;
        algaeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        m_Algae.getConfigurator().apply(algaeConfigs);

        m_Coral = new TalonFXS(ClawConstants.kCoral,RobotConstants.kCanivoreBusName);

        TalonFXSConfiguration coralConfigs = new TalonFXSConfiguration();

        coralConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        coralConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        coralConfigs.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;

        m_Coral.getConfigurator().apply(coralConfigs);
    }

    public static class PeriodicIO{
        public double coralCurrent = 0;
        public double algaeCurrent = 0;
        
        public double coralOutput=0;
        public double algaeOutput=0;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setCoralOutput(double output){
        m_PeriodicIO.coralOutput = output;
    }

    public void setAlgaeOutput(double output){
        m_PeriodicIO.algaeOutput = output;
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
        SmartDashboard.putNumber("Claw/AlgaeCurrent", m_PeriodicIO.algaeCurrent);
        SmartDashboard.putNumber("Claw/CoralCurrent", m_PeriodicIO.coralCurrent);

        SignalLogger.writeDouble("Claw/AlgaeCurrent", m_PeriodicIO.algaeCurrent);
        SignalLogger.writeDouble("Claw/CoralCurrent", m_PeriodicIO.coralCurrent);
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.coralCurrent = m_Coral.getStatorCurrent().getValueAsDouble();
        m_PeriodicIO.algaeCurrent = m_Algae.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        m_Algae.setControl(m_OutputRequest.withOutput(m_PeriodicIO.algaeOutput));
        m_Coral.setControl(m_OutputRequest.withOutput(m_PeriodicIO.coralOutput));
    }
}
