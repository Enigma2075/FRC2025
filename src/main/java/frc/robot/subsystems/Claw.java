package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class Claw extends SubsystemIO {

    public enum ControlMode {
        OUTPUT,
        POSITION,
        SYSID
    }

    private TalonFX m_Coral;
    private TalonFX m_Algae;

    //private final Output m_Output = new Output(0);

    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);

    public Claw() {
        m_Coral = new TalonFX(ClawConstants.kCoral,RobotConstants.kCanivoreBusName);
        m_Algae = new TalonFX(ClawConstants.kAlgae,RobotConstants.kCanivoreBusName);
    }

    public enum State {
        L1(0),
        L2(0),
        L3(0),
        L4(0),
        INTAKE(0);

        public final double clawAngle;

        private State(double clawAngle) {
            this.clawAngle = clawAngle;
        }
    }

    public static class PeriodicIO{
        public ControlMode controlMode = ControlMode.OUTPUT;

        double enc = 0;

        public double targetOutput = 0;
        public double lastTargetOutput;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

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

    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.enc = m_Coral.getPosition().getValueAsDouble();
        m_PeriodicIO.enc = m_Algae.getPosition().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (m_PeriodicIO.controlMode) {
            case OUTPUT:
                if(m_PeriodicIO.targetOutput != m_PeriodicIO.lastTargetOutput) {
                    m_Algae.setControl(m_OutputRequest.withOutput(m_PeriodicIO.targetOutput));
                    m_PeriodicIO.lastTargetOutput = m_PeriodicIO.targetOutput;
                }
                break;

            
            }
    }
}
