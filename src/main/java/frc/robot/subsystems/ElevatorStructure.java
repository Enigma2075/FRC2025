package frc.robot.subsystems;

public class ElevatorStructure extends SubsystemIO {
    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private final Claw m_Claw;

    public ElevatorStructure(Arm arm, Wrist wrist, Claw claw) {
        m_Arm = arm;
        m_Wrist = wrist;
        m_Claw = claw;
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

    }

}
