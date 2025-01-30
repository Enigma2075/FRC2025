package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Claw extends SubsystemIO{

    private TalonFX m_coral;
    private TalonFX m_algae;

    public Claw(){
        m_coral = new TalonFX(ClawConstants.kCoral,RobotConstants.kCanivoreBusName);
        m_algae = new TalonFX(ClawConstants.kAlgae,RobotConstants.kCanivoreBusName);
    }

    public static class PeriodicIO{

    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

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

    }
}
