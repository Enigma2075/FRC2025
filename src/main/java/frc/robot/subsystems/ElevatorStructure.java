package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.states.ElevatorStructurePosition;

public class ElevatorStructure extends SubsystemIO {
    private final Elevator m_Elevator;
    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private final Claw m_Claw;

    public static final ElevatorStructurePosition IntakeCoral = new ElevatorStructurePosition(0, 0, 0, "IntakeCoral");
    public static final ElevatorStructurePosition IntakeAlgae = new ElevatorStructurePosition(0, 0, 0, "IntakeAlgae");
    public static final ElevatorStructurePosition ScoreNet = new ElevatorStructurePosition(0, 0, 0, "ScoreNet");
    public static final ElevatorStructurePosition ScoreProcessor = new ElevatorStructurePosition(0, 0, 0, "ScoreProcessor");
    public static final ElevatorStructurePosition L4 = new ElevatorStructurePosition(0, 0, 0, "L4");
    public static final ElevatorStructurePosition L3 = new ElevatorStructurePosition(0, 0, 0, "L3");
    public static final ElevatorStructurePosition L2 = new ElevatorStructurePosition(0, 0, 0, "L2");
    public static final ElevatorStructurePosition L1 = new ElevatorStructurePosition(0, 0, 0, "L1");

    public ElevatorStructure(Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
        m_Elevator = elevator;
        m_Arm = arm;
        m_Wrist = wrist;
        m_Claw = claw;
        
        applyPosition();
    }
   
    private static class PeriodicIO {
        public ElevatorStructurePosition targetPosition = L4;

        public ElevatorStructurePosition lastPosition;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    private void applyPosition() {
        if(m_PeriodicIO.targetPosition != m_PeriodicIO.lastPosition) {
            m_PeriodicIO.lastPosition = m_PeriodicIO.targetPosition;
            applyPosition(m_PeriodicIO.targetPosition);
        }
    }

    private void applyPosition(ElevatorStructurePosition state) {
        m_Elevator.setHeight(state.ElevatorHeight);
        m_Arm.setAngle(state.ArmAngle);
        m_Wrist.setAngle(state.WristAngle);
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
        SmartDashboard.putString("ElevatorStructure/TargetState", m_PeriodicIO.targetPosition.Name);
        SmartDashboard.putString("ElevatorStructure/LastState", m_PeriodicIO.lastPosition.Name);
        
        SignalLogger.writeString("ElevatorStructure/TargetState", m_PeriodicIO.targetPosition.Name);
        SignalLogger.writeString("ElevatorStructure/LastState", m_PeriodicIO.lastPosition.Name);
    }

}
