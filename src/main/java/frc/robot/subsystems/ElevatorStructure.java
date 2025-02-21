package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.states.ElevatorStructurePosition;

public class ElevatorStructure extends SubsystemIO {
    private final Elevator m_Elevator;
    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private final Claw m_Claw;

    public static final ElevatorStructurePosition Starting = new ElevatorStructurePosition(7, 90, 0, "Starting");
    public static final ElevatorStructurePosition BargeBack = new ElevatorStructurePosition(60, 125, -75, "BargeBack");
    public static final ElevatorStructurePosition BargeFront = new ElevatorStructurePosition(60, 87, -160, "BargeFront");
    public static final ElevatorStructurePosition IntakeCoralRear = new ElevatorStructurePosition(15, 118, -153, "IntakeCoralRear");
    public static final ElevatorStructurePosition IntakeCoralFront = new ElevatorStructurePosition(11.5, 76, 137, "IntakeCoralFront");
    public static final ElevatorStructurePosition IntakeAlgaeHighRear = new ElevatorStructurePosition(30, 52, 175, "IntakeAlgaeHighRear");
    public static final ElevatorStructurePosition IntakeAlgaeHighFront = new ElevatorStructurePosition(47, 48, 175, "IntakeAlgaeHighFront");
    public static final ElevatorStructurePosition L4Front = new ElevatorStructurePosition(60, 66, 65, "L4Front");
    public static final ElevatorStructurePosition L3Front = new ElevatorStructurePosition(39, 66, 65, "L3Front");
    public static final ElevatorStructurePosition L2Front = new ElevatorStructurePosition(22, 66, 65, "L2Front");
    public static final ElevatorStructurePosition L1Front = new ElevatorStructurePosition(7, 66, 85, "L1Front");
    public static final ElevatorStructurePosition L4Rear = new ElevatorStructurePosition(7, 127, -52, "L4Rear");
    public static final ElevatorStructurePosition L3Rear = new ElevatorStructurePosition(60, 105, -70, "L3Rear");
    public static final ElevatorStructurePosition L2Rear = new ElevatorStructurePosition(33.5, 105, -70, "L2Rear");
    public static final ElevatorStructurePosition L1Rear = new ElevatorStructurePosition(18, 115, -90.5, "L1Rear");
    public static final ElevatorStructurePosition Climb = new ElevatorStructurePosition(7, 115, 0, "Climb");

    public ElevatorStructure(Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
        m_Elevator = elevator;
        m_Arm = arm;
        m_Wrist = wrist;
        m_Claw = claw;
        
        applyPosition();
    }

    public Command moveToPosition(ElevatorStructurePosition position) {
        return run(() -> {
            m_PeriodicIO.targetPosition = position;
            applyPosition();
        }).until(() -> isAtPosition());
    }
    
    public Command moveToPosition(ElevatorStructurePosition... positions) {
        Command command = null;
        for (ElevatorStructurePosition p : positions) { 
            if (command == null) { 
                command = moveToPosition(p);
            } else { 
                command = command.andThen(moveToPosition(p)); 
            } 
        }
        return command;
    }

    public Command moveToClimb() {
        return moveToPosition(Climb);
    }

    public Command moveToL4(boolean front) {
        if(front) {
            return moveToPosition(L4Front);
        } else {
            return moveToPosition(L4Front);
        }
    }

    public Command moveToIntakeCoral(boolean front) {
        if(front) {
            return moveToPosition(IntakeCoralFront);
        } else {
            return moveToPosition(IntakeCoralRear);
        }
    }

    public Command moveToStarting() {
        return moveToPosition(Starting);
    }

    public boolean isAtPosition() {
        return isAtPosition(m_PeriodicIO.targetPosition);
    }

    public boolean isAtPosition(ElevatorStructurePosition position) {
        return true;//m_Arm.isAtPosition(position.ArmAngle) && m_Wrist.isAtPosition(position.WristAngle) && m_Elevator.isAtPosition(position.ElevatorHeight);
    }
   
    private static class PeriodicIO {
        public ElevatorStructurePosition targetPosition = Starting;

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
        m_Arm.setDegrees(state.ArmAngle);
        m_Wrist.setDegrees(state.WristAngle);
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
