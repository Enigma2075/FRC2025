package frc.robot.subsystems.states;

import frc.robot.subsystems.ArmConstants;
import frc.robot.subsystems.ElevatorConst;
import frc.robot.subsystems.WristConstants;

public class ElevatorStructurePosition {
    public double ElevatorHeight; //height 
    public double ArmAngle; //angles
    public double WristAngle; //angles
    public String Name;

    public ElevatorStructurePosition(double elevator, double arm, double wrist, String name){
        this.ElevatorHeight = elevator;
        this.ArmAngle = arm;
        this.WristAngle = wrist;
        this.Name = name;
    }

    public ElevatorStructurePosition(ElevatorStructurePosition other){
        this.ElevatorHeight = other.ElevatorHeight;
        this.ArmAngle = other.ArmAngle;
        this.WristAngle = other.WristAngle;
        this.Name = other.Name;
    }

    //default position
    private ElevatorStructurePosition(){
        this(0,0,0, "");
    }

    /**
     * @return height of the end effector from the floor 
     */
    public double getEndEffectorHeight(){
        double z = ElevatorConst.kHeightOffset;
        z += this.ElevatorHeight;
        //z += ArmConstants.kArmLength * Math.sin(this.ArmAngle);
        //z += WristConstants.kWristToEndEffectorLength * Math.sin(this.WristAngle);
        return z;
    }

    public String ToString(){
        return "ElevatorState{" +
                ", elevator = " + this.ElevatorHeight +
                ", arm = " + this.ArmAngle +
                ", wrist = " + this.WristAngle +
                "}";
    }
}
