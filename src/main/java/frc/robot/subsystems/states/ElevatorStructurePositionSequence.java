package frc.robot.subsystems.states;

import frc.robot.subsystems.ArmConstants;
import frc.robot.subsystems.ElevatorConst;
import frc.robot.subsystems.WristConstants;

public class ElevatorStructurePositionSequence {
    private final ElevatorStructurePosition[] positions;

    public ElevatorStructurePositionSequence(ElevatorStructurePosition... positions){
        this.positions = positions;
    }

    public ElevatorStructurePosition[] getPositions() {
        return positions;
    }
}
