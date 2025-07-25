package frc.robot.subsystems;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringSides;
import frc.robot.subsystems.Claw.AlgaeModes;
import frc.robot.subsystems.Claw.CoralModes;
import frc.robot.subsystems.states.ElevatorStructurePosition;
import frc.robot.subsystems.states.ElevatorStructurePositionSequence;
import frc.robot.util.Utils;

public class ElevatorStructure extends SubsystemIO {
    // Intake physical position 35 high, 5.5 away from wall.


    private final Elevator m_Elevator;
    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private final Claw m_Claw;

    public static final ElevatorStructurePosition AutoStart = new ElevatorStructurePosition(7.5, 85, -100, "AutoStart"); //7.5
    public static final ElevatorStructurePosition Starting = new ElevatorStructurePosition(7.5, 85, 100, "Starting"); //7.5
    public static final ElevatorStructurePosition IntakeCoralEnd = new ElevatorStructurePosition(16, 85, 137, "IntakeCoralEnd"); //7.5
    public static final ElevatorStructurePosition StartingWithAlgae = new ElevatorStructurePosition(10, 85, 100, "StartingWithAlgae"); //7.5
    public static final ElevatorStructurePosition FloorIntake = new ElevatorStructurePosition(16, 85, 100, "StartingWithAlgae"); //7.5

    public static final ElevatorStructurePosition PickupAlgae = new ElevatorStructurePosition(14, 90, -110, "PickuptAlgae");
    public static final ElevatorStructurePosition PickupAlgaeEnd = new ElevatorStructurePosition(7.5, 90, 100, "PickuptAlgaeEnd");
    
    public static final ElevatorStructurePosition StoreAlgae = new ElevatorStructurePosition(10, 105, -100, "StoreAlgae");
    public static final ElevatorStructurePosition StoreAlgaeHeight = new ElevatorStructurePosition(16, 90, -100, "StoreAlgaeHeight");
    public static final ElevatorStructurePosition StoreAlgaeSpin = new ElevatorStructurePosition(16, 90, 100, "StoreAlgaeHeight");
    
    public static final ElevatorStructurePosition GrabAlgae = new ElevatorStructurePosition(7.5, 94, -134, "GrabAlgae");
    public static final ElevatorStructurePosition GrabAlgaeHeight = new ElevatorStructurePosition(12, 90, 100, "GrabAlgaeHeight");
    public static final ElevatorStructurePosition GrabAlgaeRotate = new ElevatorStructurePosition(12, 90, -83, "GrabAlgaeRotate");
    public static final ElevatorStructurePosition GrabAlgaeRotateBoth = new ElevatorStructurePosition(12, 116, -83, "GrabAlgaeRotateBoth");

    public static final ElevatorStructurePosition BargeRear = new ElevatorStructurePosition(Utils.getValue(68.5, 65.5), 90, 60, "BargeBack");
    public static final ElevatorStructurePosition BargeRearScore = new ElevatorStructurePosition(Utils.getValue(68.5, 65.5), 90, 40, "BargeBackScore");
    public static final ElevatorStructurePosition BargeRearEnd = new ElevatorStructurePosition(30, 85, 40, "BargeBackEnd");
    public static final ElevatorStructurePosition BargeFront = new ElevatorStructurePosition(65.5, 103, 120, "BargeFront");
    
    public static final ElevatorStructurePosition IntakeCoralRear = new ElevatorStructurePosition(Utils.getValue(16.6, 16), Utils.getValue(67, 65), Utils.getValue(142, 137), "IntakeCoralRear");
    public static final ElevatorStructurePosition autoIntakeCoralRear = new ElevatorStructurePosition(Utils.getValue(16.6, 16), Utils.getValue(70, 63), Utils.getValue(142, 137), "IntakeCoralRear");
    public static final ElevatorStructurePosition IntakeCoralFront = new ElevatorStructurePosition(14.5, 118, -153, "IntakeCoralFront");
    public static final ElevatorStructurePosition IntakeCoralFrontEnd = new ElevatorStructurePosition(15, 90, 100, "IntakeCoralFrontEnd");
    
    public static final ElevatorStructurePosition IntakeAlgaeHighFrontStart = new ElevatorStructurePosition(38, 130, 156, "IntakeAlgaeHighFrontStart");
    public static final ElevatorStructurePosition IntakeAlgaeHighFront = new ElevatorStructurePosition(38, 90, 156, "IntakeAlgaeHighFront");
    public static final ElevatorStructurePosition IntakeAlgaeHighFrontEnd = new ElevatorStructurePosition(2, 85, 100, "IntakeAlgaeHighFrontEnd");
    public static final ElevatorStructurePositionSequence AlgaeHighFrontSequence = new ElevatorStructurePositionSequence(IntakeAlgaeHighFrontStart);
    public static final ElevatorStructurePositionSequence IntakeAlgaeHighFrontSequence = new ElevatorStructurePositionSequence(IntakeAlgaeHighFront, IntakeAlgaeHighFrontEnd);
    
    public static final ElevatorStructurePosition StoreAlgaeMove = new ElevatorStructurePosition(15, 90, 40, "IntakeAlgaeHighFrontEndMove");
    public static final ElevatorStructurePositionSequence StoreAlgaeSequence = new ElevatorStructurePositionSequence(StoreAlgaeMove, GrabAlgaeRotateBoth, StoreAlgae);
    
    public static final ElevatorStructurePosition IntakeAlgaeHighRear = new ElevatorStructurePosition(44, 110, 175, "IntakeAlgaeHighRear");
    public static final ElevatorStructurePositionSequence IntakeAlgaeHighRearSequence = new ElevatorStructurePositionSequence(IntakeAlgaeHighRear);
    public static final ElevatorStructurePositionSequence AlgaeHighRearSequence = new ElevatorStructurePositionSequence(IntakeAlgaeHighRear);
    
    public static final ElevatorStructurePosition IntakeAlgaeLowFrontStart = new ElevatorStructurePosition(7.5, 82, 157, "IntakeAlgaeLowFrontStart");
    public static final ElevatorStructurePosition IntakeAlgaeLowFrontTuck = new ElevatorStructurePosition(7.5, 91, 157, "IntakeAlgaeLowFront");
    public static final ElevatorStructurePosition IntakeAlgaeLowFront = new ElevatorStructurePosition(21, 134, 139, "IntakeAlgaeLowFront");
    public static final ElevatorStructurePosition AutoIntakeAlgaeLowFront = new ElevatorStructurePosition(21, 90, 139, "IntakeAlgaeLowFront");
    public static final ElevatorStructurePosition IntakeAlgaeLowFrontGrab = new ElevatorStructurePosition(19, 90, 139, "IntakeAlgaeLowFrontGrab");
    public static final ElevatorStructurePosition IntakeAlgaeLowFrontGrabAuto = new ElevatorStructurePosition(19, 100, 139, "IntakeAlgaeLowFrontGrabAuto");
    public static final ElevatorStructurePosition IntakeAlgaeLowFrontEnd = new ElevatorStructurePosition(2, 85, 100, "IntakeAlgaeLowFrontEnd");
    
    public static final ElevatorStructurePositionSequence AlgaeLowFrontSequence = new ElevatorStructurePositionSequence(IntakeAlgaeLowFront);
    
    public static final ElevatorStructurePositionSequence IntakeAlgaeLowFrontSequence = new ElevatorStructurePositionSequence(IntakeAlgaeLowFrontGrab, IntakeAlgaeLowFrontEnd);
    
    public static final ElevatorStructurePosition IntakeAlgaeLowRear = new ElevatorStructurePosition(30, 110, 175, "IntakeAlgaeLowRear");
    public static final ElevatorStructurePositionSequence AlgaeLowRearSequence = new ElevatorStructurePositionSequence(IntakeAlgaeLowRear);
    public static final ElevatorStructurePositionSequence IntakeAlgaeLowRearSequence = new ElevatorStructurePositionSequence(IntakeAlgaeLowRear);
    
    public static final ElevatorStructurePosition L4Rear = new ElevatorStructurePosition(63, 66, 65, "L4Rear");
    public static final ElevatorStructurePosition L3Rear = new ElevatorStructurePosition(39, 66,  65, "L3Rear");
    public static final ElevatorStructurePosition L2Rear = new ElevatorStructurePosition(22, 66, 65, "L2Rear");
    public static final ElevatorStructurePosition L1Rear = new ElevatorStructurePosition(7.5, 66, 85, "L1Rear");
    
    public static final ElevatorStructurePosition L4Front = new ElevatorStructurePosition(Utils.getValue(68.5, 65.5), 120, -60, "L4Front");
    public static final ElevatorStructurePosition autoL4Front = new ElevatorStructurePosition(Utils.getValue(68.5, 65.5), 120, -60, "L4Front");
    public static final ElevatorStructurePosition L3Front = new ElevatorStructurePosition(35, 110, -80, "L3Front");
    public static final ElevatorStructurePosition L2Front = new ElevatorStructurePosition(18, 109, -80, "L2Front");
    public static final ElevatorStructurePosition L1Front = new ElevatorStructurePosition(7.5, 125, -140, "L1Front");
    
    public static final ElevatorStructurePosition Climb = new ElevatorStructurePosition(7.5, 122, 75, "Climb");

    private enum QueueModes {ALGAE, CORAL, NONE};

    private QueueModes m_QueueMode = QueueModes.NONE;
    private boolean m_CoralPositionPressed = false;
    private boolean m_AlgaePositionPressed = false;
    private Supplier<Command> m_NextCommand = null;
    private Function<Intake.States, Command> m_moveIntake = null;

    private Supplier<Command> m_AlgaeIntakeEndCommand = null;
    
    public ElevatorStructure(Elevator elevator, Arm arm, Wrist wrist, Claw claw, Function<Intake.States, Command> moveIntakeIntake) {
        m_Elevator = elevator;
        m_Arm = arm;
        m_Wrist = wrist;
        m_Claw = claw;
        m_moveIntake = moveIntakeIntake;
        
        applyPosition();
    }

    public Command moveToPosition(ElevatorStructurePosition frontPosition, ElevatorStructurePosition rearPosition) {
        return moveToPosition(frontPosition, rearPosition, null);
    }

    public boolean requestingStartingPosition() {
        return m_PeriodicIO.targetPosition == Starting;
    }

    private void applyPositionBySide(ElevatorStructurePosition front, ElevatorStructurePosition back) {
        if(RobotState.scoringSide == ScoringSides.FRONT) {
            m_PeriodicIO.targetPosition = front;
        }
        else {
            m_PeriodicIO.targetPosition = back;    
        }
        applyPosition();
    }

    public Command moveToPosition(ElevatorStructurePosition front, ElevatorStructurePosition back, Runnable action) {
        return run(() -> {
            if(action != null) {
                action.run();
            }

            applyPositionBySide(front, back);
        });
    }

    public Command moveToPosition(ElevatorStructurePosition position) {
        return moveToPosition(false, position);
    }

    public Command moveToPosition(boolean wait, ElevatorStructurePosition position) {
        return run(() -> {
            m_PeriodicIO.targetPosition = position;
            applyPosition();
        }).until(() -> !wait || isAtPosition());
    }

    public Command moveToPositions(ElevatorStructurePosition... positions) {
        return moveToPositions(true, positions);
    }

    public Command moveToPositions(Runnable action, ElevatorStructurePosition... positions) {
        return moveToPositions(true, action, positions);
    }

    public Command moveToPositions(boolean wait, ElevatorStructurePosition... positions)
    {
        return moveToPositions(wait, null, positions);
    }

    public Command moveToPositions(boolean wait, Runnable action, ElevatorStructurePosition... positions) {
        Command command = null;

        if(action != null){
            command = runOnce(action);
        }

        for (ElevatorStructurePosition p : positions) { 
            if (command == null) { 
                command = moveToPosition(wait, p);
            } else { 
                command = command.andThen(moveToPosition(wait, p)); 
            } 
        }
        return command;
    }

    public Command moveToPositionsSide(ElevatorStructurePositionSequence front, ElevatorStructurePositionSequence back)
        {
            return moveToPositionsSide(true, null, front, back);
        }

        public Command moveToPositionsSide(Runnable action, ElevatorStructurePositionSequence front, ElevatorStructurePositionSequence back)
        {
            return moveToPositionsSide(true, action, front, back);
        }

        public Command moveToPositionsSide(boolean wait, ElevatorStructurePositionSequence front, ElevatorStructurePositionSequence back)
    {
        return moveToPositionsSide(wait, null, front, back);
    }
    
    public Command moveToPositionsSide(boolean wait, Runnable action, ElevatorStructurePositionSequence front, ElevatorStructurePositionSequence back) {
        if(RobotState.scoringSide == ScoringSides.FRONT) {
            return moveToPositions(wait, action, front.getPositions());
        }
        else {
            return moveToPositions(wait, action, back.getPositions());
        }
    }

    public Command moveToClimb() {
        return moveToPosition(Climb);
    }

    public Command clearCoralPress() {
        return run(() -> m_CoralPositionPressed = false);
    }

    public Command clearAlgaePress() {
        return run(() -> m_AlgaePositionPressed = false);
    }

    public Command moveToBargeCommand() {
        return moveToPosition(BargeRear, BargeRear);
    }

    private Command moveToCoralLevel(Supplier<Command> movement) {
        return runOnce(() -> {
            m_CoralPositionPressed = true;
            if(m_QueueMode == QueueModes.NONE || !m_AlgaePositionPressed) {
                m_QueueMode = QueueModes.CORAL;
            }
            else if(m_QueueMode != QueueModes.CORAL) {
                m_NextCommand = movement;
            }
        })
        .andThen(movement.get().unless(() -> m_QueueMode != QueueModes.NONE && m_QueueMode != QueueModes.CORAL))
        .andThen(run(() -> {}));
    }

    public Command autoMoveToL4Command() {
        return runOnce(() -> m_Elevator.setOverrideVelocity(true)).andThen(moveToPosition(false, autoL4Front)).finallyDo(() -> m_Elevator.setOverrideVelocity(false))
        .finallyDo(()-> {
            m_CoralPositionPressed = false;
        });
    }

    public Command autoMoveToL4AlgaeCommand() {
        return runOnce(() -> m_Elevator.setOverrideVelocity(false)).andThen(moveToPositions(L4Front)).finallyDo(() -> m_Elevator.setOverrideVelocity(false))
        .finallyDo(()-> {
            m_CoralPositionPressed = false;
        });
    }

    public Command moveToL4Command(boolean slow) {
        return moveToCoralLevel(() -> { 
            Command cmd = moveToPosition(L4Front, L4Rear);
            if(slow) {
                cmd = runOnce(() -> m_Elevator.setOverrideVelocity(true)).andThen(cmd).finallyDo(() -> m_Elevator.setOverrideVelocity(false));
            }
            else {
                cmd = runOnce(() -> m_Elevator.setOverrideVelocity(false)).andThen(cmd);
            }
            return cmd;
        })
        .finallyDo(()-> {
            m_CoralPositionPressed = false;
        });
    }

    public void clearPress(){
        m_CoralPositionPressed = false;
        m_AlgaePositionPressed = false;
    }

    public Command moveToL4Command() {
        return moveToCoralLevel(() -> { return moveToPosition(L4Front, L4Rear);});
    }

    public Command moveToL3Command() {
        return moveToCoralLevel(() -> { return moveToPosition(L3Front, L3Rear);});
    }

    public Command moveToL2Command() {
        return moveToCoralLevel(() -> { return moveToPosition(L2Front, L2Rear);});
    }

    public Command moveToL1Command() {
        return moveToCoralLevel(() -> { return moveToPosition(L1Front, L1Rear);});
    }

    public Command moveToStartingCommand() {
        return moveToPosition(Starting);
    }

    public Command moveToFloorIntake() {
        return moveToPosition(FloorIntake);
    }

    public Command moveToAlgaeStartingCommand() {
        return moveToPosition(StartingWithAlgae);
    }

    public Command moveToIntakeEndCommand(){
        return moveToPositions(IntakeCoralEnd,Starting);
    }

    public Command moveToEndBargeCommand() {
        return moveToPositions(BargeRearEnd,Starting);
    }

    public Command intakeCoralEndCommand() {
        return moveToPositions(IntakeCoralFrontEnd, Starting);
    }

    public void applyAutoStartPosition() {
        applyPosition(AutoStart);
    }

    public Command intakeCoralCommand() {
        return run(() -> { 
            m_Claw.setCoralMode(CoralModes.INTAKE);
            applyPositionBySide(IntakeCoralRear, IntakeCoralFront);
        });
    }

    public Command autoIntakeCoralCommand() {
        return run(() -> { 
            m_Elevator.setOverrideVelocity(false);
            m_Claw.setCoralMode(CoralModes.INTAKE);
            applyPositionBySide(autoIntakeCoralRear, IntakeCoralFront); 
        });
    }

    public Command stopCoralCommmand(){
        return runOnce(()-> m_Claw.setCoralMode(CoralModes.STOP));
    }

    public Command holdCoralCommand(){
        return runOnce(()-> m_Claw.setCoralMode(CoralModes.HOLD));
    }

    public Command handoffAlgaeCommand() {
        return moveToPositions(GrabAlgaeHeight, GrabAlgaeRotate, GrabAlgae)
        .andThen(run(() -> { m_Claw.setAlgaeMode(AlgaeModes.INTAKE, true); }).withTimeout(1));
    }

    public Command pickupAlgaeCommand() {
        return moveToPositions(() -> {
            //m_Wrist.setOverrideVelocity(true); 
            m_Claw.setAlgaeMode(AlgaeModes.INTAKE, true);}, PickupAlgae, PickupAlgaeEnd);
    }

    public Command scoreBargeCommand() {
        return moveToPosition(false, BargeRearScore).andThen(outtakeAlgaeCommand());
    }

    public Command autoScoreBargeCommand() {
        return moveToPosition(false, BargeRearScore).andThen(autoOuttakeAlgaeCommand());
    }

    public Command outtakeAlgaeCommand() {
        return runOnce(() -> m_Claw.setAlgaeMode(AlgaeModes.OUTTAKE))
            .andThen(Commands.waitUntil(() -> !m_Claw.hasAlgae()).andThen(Commands.waitSeconds(.25))) // wait for algae to be out
            .andThen(moveToEndBargeCommand());
    }

    public Command autoOuttakeAlgaeCommand() {
        return runOnce(() -> m_Claw.setAlgaeMode(AlgaeModes.OUTTAKE))
            .andThen(Commands.waitUntil(() -> !m_Claw.hasAlgae()).andThen(Commands.waitSeconds(.25))); // wait for algae to be out
            
    }

    public Command intakeAlgaeCommand() {
        return runOnce( () -> {
            if(m_AlgaeIntakeEndCommand != null) {
               CommandScheduler.getInstance().schedule(m_AlgaeIntakeEndCommand.get());
            }
            else {
                CommandScheduler.getInstance().schedule(run(() -> m_Claw.setAlgaeMode(AlgaeModes.INTAKE)));
            }
        });
    }

    public Command autoIntakeAlgaeCommand() {
        return runOnce(() -> m_Claw.setAlgaeMode(AlgaeModes.INTAKE)).andThen(Commands.waitSeconds(.05)).andThen(intakeAlgaeHighCommand());
        //return runOnce(() -> m_Claw.setAlgaeMode(AlgaeModes.INTAKE)).andThen(Commands.waitSeconds(.05));
    }

    //private Command run

    public Command autoLastOuttakeCoralCommand() {
        return runOnce(() -> {
            m_Claw.setCoralMode(CoralModes.OUTTAKE);
            }
        ).andThen(moveToPosition(false, Starting));
    }

    public Command autoOuttakeCoralCommand() {
        return new WaitUntilCommand(() -> isAtPosition()).andThen(Commands.waitSeconds(.2)).andThen(runOnce(() -> {
            m_Claw.setCoralMode(CoralModes.OUTTAKE);
            }
        )).andThen(Commands.waitSeconds(.2));
    }

    public Command autoOuttakeL4Command() {
        return runOnce(() -> {
            m_Claw.setCoralMode(CoralModes.OUTTAKE);
            }
        );
    }
    
    public Command autoOuttakeCoralToAlgaeLowCommand() {
        return new WaitUntilCommand(() -> isAtPosition()).andThen(runOnce(() -> {
            m_Claw.setCoralMode(CoralModes.OUTTAKE);
            }
        )).andThen(Commands.waitSeconds(.2).until(() -> !m_Claw.hasCoral())).andThen(autoMoveToAlgaeLowCommand());
    }
    
    public Command outtakeCoralCommand() {
        return run(() -> {
            var timeout = .25;
            if(m_PeriodicIO.targetPosition == L1Front) {
                m_Claw.setCoralMode(CoralModes.OUTTAKE_SLOW);
                timeout = 1;
            }
            else {
                m_Claw.setCoralMode(CoralModes.OUTTAKE);
            }

            if(m_NextCommand != null) {
                CommandScheduler.getInstance().schedule(Commands.waitSeconds(timeout).andThen(m_NextCommand.get()).finallyDo(() -> {m_NextCommand = null; m_QueueMode = QueueModes.NONE;}));
            }
            else {
                CommandScheduler.getInstance().schedule(Commands.waitSeconds(timeout).andThen(moveToPosition(false, Starting)));
            }
        });
    }
    
    public Command storeAlgaeCommand() {
        return moveToPositions(StoreAlgaeSequence.getPositions())
            .andThen(run(() -> m_Claw.setAlgaeMode(AlgaeModes.OUTTAKE)).until(() -> !m_Claw.hasAlgae()))
            .andThen(moveToPositions(StoreAlgaeHeight, StoreAlgaeSpin, StartingWithAlgae));
    }

    public Command moveToAlgaeHighCommand() {
        Supplier<Command> movement = () -> { return runOnce(() -> {
                m_Wrist.setOverrideVelocity(true);
                m_AlgaeIntakeEndCommand = () -> intakeAlgaeHighCommand();

            })
            .andThen(moveToPositionsSide(AlgaeHighFrontSequence, AlgaeHighRearSequence));};

        return runOnce(() -> {
            m_AlgaePositionPressed = true;
            
            if(m_QueueMode == QueueModes.NONE || !m_CoralPositionPressed) {
                m_QueueMode = QueueModes.ALGAE;
            }
            else if(m_QueueMode == QueueModes.CORAL) {
                m_NextCommand = movement;
            }
        })
        .andThen(movement.get().unless(() -> m_QueueMode == QueueModes.CORAL))
        .andThen(run(() -> {}))
        .finallyDo(() -> {
            m_Wrist.setOverrideVelocity(false);
        });
    }

    public Command intakeAlgaeHighCommand() {
        return runOnce(() -> m_Wrist.setOverrideVelocity(false))
            .andThen(moveToPositionsSide(() -> { m_Claw.setAlgaeMode(AlgaeModes.INTAKE, true);}, IntakeAlgaeHighFrontSequence, IntakeAlgaeHighRearSequence)).withTimeout(.75)
            .andThen(new ConditionalCommand( runOnce(() -> {CommandScheduler.getInstance().schedule(m_NextCommand.get().finallyDo(() -> {m_NextCommand = null;}));}) , moveToStartingCommand(), () -> m_NextCommand != null))//m_moveIntake.apply(Intake.States.HANDOFFALGAE).alongWith(storeAlgaeCommand()).finallyDo((() -> CommandScheduler.getInstance().schedule(m_moveIntake.apply(Intake.States.DEFAULT)))) , () -> m_NextCommand != null))
            .finallyDo(() -> m_Wrist.setOverrideVelocity(false));
    }

    public Command autoMoveToAlgaeLowCommand() {
        return moveToPosition(false, IntakeAlgaeLowFront);
    }

    public Command auto1MoveToAlgaeLowCommand() {
       return moveToPosition(true, IntakeAlgaeLowFront).onlyWhile(() -> !m_Claw.hasAlgae()).andThen(moveToAlgaeLowFrontGrab());
    }

    public Command moveToAlgaeLowFrontGrab() {
        return moveToPosition(true, IntakeAlgaeLowFrontGrabAuto).withTimeout(2);
    }


    public Command moveToAlgaeLowCommand() {
        Supplier<Command> movement = () -> { return runOnce(() -> {
                //m_Wrist.setOverrideVelocity(true);
                m_AlgaeIntakeEndCommand = () -> intakeAlgaeLowCommand().finallyDo(() -> m_AlgaeIntakeEndCommand = null);
            })
            .andThen(moveToPositionsSide(AlgaeLowFrontSequence, AlgaeLowRearSequence));};

        return runOnce(() -> {
            m_AlgaePositionPressed = true;
            if(m_QueueMode == QueueModes.NONE || !m_CoralPositionPressed) {
                m_QueueMode = QueueModes.ALGAE;
            }
            else if(m_QueueMode == QueueModes.CORAL) {
                m_NextCommand = movement;
            }
        })
        .andThen(movement.get().unless(() -> m_QueueMode == QueueModes.CORAL));
        //.andThen(run(() -> {}))
        //.finallyDo(() -> {
        //    m_Wrist.setOverrideVelocity(false);
        //});
    }

    public Command intakeAlgaeLowCommand() {
        return runOnce(() -> m_Wrist.setOverrideVelocity(false))
            .andThen(moveToPositionsSide(() -> { m_Claw.setAlgaeMode(AlgaeModes.INTAKE, true);}, IntakeAlgaeLowFrontSequence, IntakeAlgaeLowRearSequence)).withTimeout(.75)
            .andThen(new ConditionalCommand( runOnce(() -> {CommandScheduler.getInstance().schedule(m_NextCommand.get().finallyDo(() -> {m_NextCommand = null;}));}), moveToStartingCommand(), () -> m_NextCommand != null))//m_moveIntake.apply(Intake.States.HANDOFFALGAE).alongWith(storeAlgaeCommand()).finallyDo((() -> CommandScheduler.getInstance().schedule(m_moveIntake.apply(Intake.States.DEFAULT)))) , () -> m_NextCommand != null))
            .finallyDo(() -> m_Wrist.setOverrideVelocity(false));
    }

    // public Command intake() {
    //     return runOnce(()-> {
    //         if(m_PeriodicIO.targetPosition == IntakeCoralFront || m_PeriodicIO.targetPosition == IntakeCoralRear) {
    //             m_Claw.setCoralMode(CoralModes.INTAKE);
    //         }
    //         else if (m_PeriodicIO.targetPosition == L4Front) {
    //             applyPosition();
    //         }
    //     });
    // }

    public Command defaultCommand() {
        return run(() -> { 
            // m_Claw.setCoralOutput(.1);
            // m_Claw.setAlgaeOutput(.2);
        });
    }

    public boolean isAtPosition() {
        return isAtPosition(m_PeriodicIO.targetPosition);
    }

    public boolean isAtPosition(ElevatorStructurePosition position) {
        return m_Arm.isAtPosition(position.ArmAngle) && m_Wrist.isAtPosition(position.WristAngle) && m_Elevator.isAtPosition(position.ElevatorHeight);
    }
   
    private static class PeriodicIO {
        public ElevatorStructurePosition targetPosition = AutoStart;

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
        SmartDashboard.putBoolean("ElevatorStructure/AlgaeIntakeEnd", m_AlgaeIntakeEndCommand == null);
        SmartDashboard.putBoolean("ElevatorStructure/AtPosition", isAtPosition());
        
        SignalLogger.writeString("ElevatorStructure/TargetState", m_PeriodicIO.targetPosition.Name);
        SignalLogger.writeString("ElevatorStructure/LastState", m_PeriodicIO.lastPosition.Name);
        SignalLogger.writeBoolean("ElevatorStructure/AtPosition", isAtPosition());
        
    }

}
