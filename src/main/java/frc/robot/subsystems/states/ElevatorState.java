package frc.robot.subsystems.states;

import frc.robot.subsystems.ArmConstants;
import frc.robot.subsystems.ElevatorConst;
import frc.robot.subsystems.WristConstants;

public class ElevatorState {
    public double elevator; //height 
    public double arm; //angles
    public double wrist; //angles

    public ElevatorState(double elevator, double arm, double wrist){
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
    }

    public ElevatorState(ElevatorState other){
        this.elevator = other.elevator;
        this.arm = other.arm;
        this.wrist = other.wrist;
    }

    //default position
    public ElevatorState(){
        this(0,0,0);
    }

    /**
     * @return height of the end effector from the floor 
     */
    public double getEndEffectorHeight(){
        double z = ElevatorConst.kDefaultElevatorHeight;
        z += this.elevator;
        z += ArmConstants.kArmLength * Math.sin(this.arm);
        z += WristConstants.kWristToEndEffectorLength * Math.sin(this.wrist);
        return z;
    }

    public String ToString(){
        return "ElevatorState{" +
                ", elevator = " + this.elevator +
                ", arm = " + this.arm +
                ", wrist = " + this.wrist +
                "}";
    }
}
