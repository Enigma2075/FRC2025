// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorStructure;
import frc.robot.subsystems.IOManager;
import frc.robot.subsystems.RobotConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionConstant;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Utils;
import frc.robot.subsystems.Climb.State;
import frc.robot.subsystems.Intake.States;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrainConstants;

public class RobotContainer {
    private double MaxSpeed = DriveTrainConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed
    private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                     // max angular velocity = 166.16
    private double CalculatedMaxSpeed = MaxSpeed;
    private double CalculatedMaxAngularRate = MaxAngularRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final RobotCentricFacingAngle driveRobotCentric = new SwerveRequest.RobotCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.001).withRotationalDeadband(MaxAngularRate * 0.001) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final RobotCentric driveRobotCentric1 = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.001).withRotationalDeadband(MaxAngularRate * 0.001) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController auxilary = new CommandXboxController(2);

    public final Drivetrain drivetrain = DriveTrainConstants.createDrivetrain();

    public final Elevator elevator = new Elevator();

    public final Climb climb = new Climb();

    public final Arm arm = new Arm();

    public final Claw claw = new Claw();

    public final Wrist wrist = new Wrist();

    public final Intake intake = new Intake();
    
    public final ElevatorStructure elevatorStructure = new ElevatorStructure(elevator, arm, wrist, claw, (state) -> intake.setStateCommand(state));
    
    public final Vision vision = new Vision(drivetrain::addVisionMeasurement,
            () -> drivetrain.getState().Pose.getRotation(), this::getAllianceSide, this::updateReefPose, this::updateTargetPose);

    private int priorityId = 0;

    private Pose2d robotPoseInTargetSpace = new Pose2d();

    private SendableChooser<Command> autoChooser;

    public IOManager ioManager;

    public boolean waitForPosition = false;
    public ReefSides currentReefSide = ReefSides.LEFT;

    public RobotContainer() {
        //drivetrain.setStateStdDevs(VecBuilder.fill(9999, 9999, 0));
        driveAtAngle.HeadingController.setP(10);
        driveAtAngle.MaxAbsRotationalRate = MaxAngularRate;

        driveRobotCentric.HeadingController.setP(10);
        driveRobotCentric.MaxAbsRotationalRate = MaxAngularRate;

        NamedCommands.registerCommand("intake", elevatorStructure.autoIntakeCoralCommand());
        NamedCommands.registerCommand("starting_position", elevatorStructure.moveToStartingCommand());
        NamedCommands.registerCommand("drive_backward_right", elevatorStructure.intakeCoralCommand().alongWith(driveBackwardCommand(true)).until(() -> claw.hasCoral()).withTimeout(2));
        NamedCommands.registerCommand("wait_for_coral", Commands.waitUntil(() ->claw.hasCoral()).withTimeout(.5));
        NamedCommands.registerCommand("drive_backward_left", elevatorStructure.intakeCoralCommand().alongWith(driveBackwardCommand(false)).until(() -> claw.hasCoral()).withTimeout(2));
        NamedCommands.registerCommand("move_to_L4", elevatorStructure.autoMoveToL4Command());
        NamedCommands.registerCommand("move_to_L4_Algae", Commands.waitUntil(() -> claw.hasAlgae()).withTimeout(1).andThen(elevatorStructure.intakeAlgaeLowCommand().until(() ->elevatorStructure.isAtPosition())).andThen(elevatorStructure.autoMoveToL4AlgaeCommand()));
        
        NamedCommands.registerCommand("intakeAlgaeA", Commands.waitUntil(() -> claw.hasAlgae()).withTimeout(1).andThen(elevatorStructure.intakeAlgaeHighCommand().until(() ->elevatorStructure.isAtPosition())));
        
        NamedCommands.registerCommand("foundTag1_left", vision.setPriorityId(21, 10).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.LEFT))));
        NamedCommands.registerCommand("foundTag1_center", vision.setPriorityId(21, 10).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.CENTER))));
        NamedCommands.registerCommand("foundTag2_left", vision.setPriorityId(22, 9).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.LEFT))));
        NamedCommands.registerCommand("foundTag2_right", vision.setPriorityId(22, 9).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.RIGHT)))); 
        NamedCommands.registerCommand("foundTag3_left", vision.setPriorityId(17, 8).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.LEFT))));
        NamedCommands.registerCommand("foundTag3_right", vision.setPriorityId(17, 8).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.RIGHT))));
        NamedCommands.registerCommand("foundTag4", vision.setPriorityId(18, 7).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.LEFT))));
        NamedCommands.registerCommand("foundTag5_left", vision.setPriorityId(19, 6).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.LEFT))));
        NamedCommands.registerCommand("foundTag5_right", vision.setPriorityId(19, 6).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.RIGHT))));
        NamedCommands.registerCommand("foundTag6", vision.setPriorityId(20, 11).andThen(Commands.waitUntil(() -> closeToTarget(ReefSides.LEFT))));
        
        NamedCommands.registerCommand("outtake1_left", vision.setPriorityId(21, 10).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralToAlgaeLowCommand())));
        NamedCommands.registerCommand("outtake1", vision.setPriorityId(21, 10).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake2_left", vision.setPriorityId(22, 9).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake2_right", vision.setPriorityId(22, 9).andThen(driveToTargetAuto(ReefSides.RIGHT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake3_left", vision.setPriorityId(17, 8).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake3_right", vision.setPriorityId(17, 8).andThen(driveToTargetAuto(ReefSides.RIGHT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake4", vision.setPriorityId(18, 7).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake5_left", vision.setPriorityId(19, 6).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake5_right", vision.setPriorityId(19, 6).andThen(driveToTargetAuto(ReefSides.RIGHT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));
        NamedCommands.registerCommand("outtake6", vision.setPriorityId(20, 11).andThen(driveToTargetAuto(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand()).andThen(Commands.waitSeconds(.05))));

        NamedCommands.registerCommand("intakeAlgae1", vision.setPriorityId(21, 10).andThen(driveToTargetAuto(ReefSides.CENTER).alongWith(elevatorStructure.auto1MoveToAlgaeLowCommand())));
        NamedCommands.registerCommand("intakeAlgae2", vision.setPriorityId(22, 9).andThen(driveToTargetAuto(ReefSides.CENTER).alongWith(Commands.waitUntil(() -> claw.hasAlgae() || isAtPositionAuto(ReefSides.CENTER)).withTimeout(.5).andThen(elevatorStructure.autoIntakeAlgaeCommand()))));
        NamedCommands.registerCommand("intakeAlgae3", vision.setPriorityId(20, 11).andThen(driveToTargetAuto(ReefSides.CENTER).alongWith(Commands.waitUntil(() -> claw.hasAlgae() || isAtPositionAuto(ReefSides.CENTER)).withTimeout(.5).andThen(elevatorStructure.autoIntakeAlgaeCommand()))));

        NamedCommands.registerCommand("intakeAlgaeMore", elevatorStructure.intakeAlgaeLowCommand());

        NamedCommands.registerCommand("intakeAlgaetoL4", vision.setPriorityId(21, 10).andThen(driveToTargetAuto(ReefSides.CENTER).andThen(elevatorStructure.autoIntakeAlgaeCommand()).andThen(Commands.waitSeconds(.05))).andThen(elevatorStructure.moveToL4Command(false)));

        NamedCommands.registerCommand("move_to_low_algae", elevatorStructure.moveToAlgaeLowCommand());
        NamedCommands.registerCommand("move_to_high_algae", moveToAlgaeHighCommand());
        NamedCommands.registerCommand("move_to_barge", elevatorStructure.moveToBargeCommand());
        
        NamedCommands.registerCommand("outtake_algae", Commands.waitUntil(() -> elevatorStructure.isAtPosition()).andThen(elevatorStructure.autoScoreBargeCommand()));
        NamedCommands.registerCommand("outtake_coral", elevatorStructure.autoOuttakeL4Command());
        NamedCommands.registerCommand("last_outtake_coral", elevatorStructure.autoLastOuttakeCoralCommand());


        NamedCommands.registerCommand("rotationTest", rotationTestCommand());

        ioManager = new IOManager(climb, elevator, arm, wrist, claw, intake, elevatorStructure, vision);

        SmartDashboard.putBoolean("PracticeBot", RobotConstants.kPracticeBot);

        configureBindings();

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Right", drivetrain.getAutoPath("Right"));
        autoChooser.addOption("Left", drivetrain.getAutoPath("Left"));
        //autoChooser.addOption("Middle", drivetrain.getAutoPath("Middle"));
        autoChooser.addOption("Straight", drivetrain.getAutoPath("Straight"));
        //autoChooser.addOption("RotationTest", drivetrain.getAutoPath("RotationTest"));
        //autoChooser.addOption("test1", drivetrain.getAutoPath("Test1"));
        //autoChooser.addOption("test2", drivetrain.getAutoPath("Test2"));
        //autoChooser.addOption("Ken", drivetrain.getAutoPath("Right-Ken"));
        autoChooser.addOption("Center", drivetrain.getAutoPath("Center"));
        autoChooser.addOption("Center-2IRI", drivetrain.getAutoPath("Center-2IRI"));
        //autoChooser.addOption("Center-2", drivetrain.getAutoPath("Center-2"));
        //autoChooser.addOption("Right-4", drivetrain.getAutoPath("Right-4"));
        //autoChooser.addOption("2path", drivetrain.getAutoPath("2path"));


        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void calculateMaxSpeed() {
        double maxSpeedReduction = .8;
        double maxAngularRateReduction = .8;

        double percentOfMaxHeight = elevator.getHeightWithoutOffset() / elevator.getMaxHeightWithoutOffset();
        double percentOfMaxHeightSquared = percentOfMaxHeight * percentOfMaxHeight;

        CalculatedMaxSpeed = MaxSpeed * (1.0 - (percentOfMaxHeightSquared * maxSpeedReduction));
        CalculatedMaxAngularRate = MaxAngularRate * (1.0 - (percentOfMaxHeightSquared * maxAngularRateReduction));

        driveAtAngle.MaxAbsRotationalRate = CalculatedMaxAngularRate;

        SmartDashboard.putNumber("Drivetrain/CalculatedMaxSpeed", CalculatedMaxSpeed);
        SmartDashboard.putNumber("Drivetrain/CalculatedMaxAngularRate", CalculatedMaxAngularRate);
    }

    private double applyExpo(double val) {
        return Math.signum(val) * val * val;
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {return defaultDrive().get();}));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

        // run sysid for intake and climb
        // operator.a().whileTrue(intake.sysIdQuasiStatic(Direction.kReverse));
        // operator.b().whileTrue(intake.sysIdDynamic(Direction.kForward));
        // operator.x().whileTrue(intake.sysIdDynamic(Direction.kForward));
        // operator.y().whileTrue(intake.sysIdQuasiStatic(Direction.kForward));

        // reset the field-centric heading on left bumper press
        // driver.leftBumper().onTrue(drivetrain.runOnce(() ->
        // drivetrain.seedFieldCentric()));

        elevatorStructure.setDefaultCommand(elevatorStructure.defaultCommand());

        // operator.povRight().whileTrue(Commands.run(() -> {
        // RobotState.scoringSide = ScoringSides.BACK;
        // }));
        // operator.povLeft().whileTrue(Commands.run(() -> {
        // RobotState.scoringSide = ScoringSides.FRONT;
        // }));

        // move to Algae High
        operator.povUp().whileTrue(elevatorStructure.moveToAlgaeHighCommand())
                .onFalse(elevatorStructure.clearAlgaePress());
        
        // move to Algae Low
        operator.povDown().whileTrue(elevatorStructure.moveToAlgaeLowCommand())
                .onFalse(elevatorStructure.clearAlgaePress());

        operator.y().whileTrue(elevatorStructure.moveToL4Command()).onFalse(elevatorStructure.clearCoralPress());
        operator.b().whileTrue(elevatorStructure.moveToL3Command()).onFalse(elevatorStructure.clearCoralPress());
        operator.a().whileTrue(elevatorStructure.moveToL2Command()).onFalse(elevatorStructure.clearCoralPress());
        operator.x().whileTrue(elevatorStructure.moveToL1Command()).onFalse(elevatorStructure.clearCoralPress());

        // Reset arm
        operator.rightStick().whileTrue(elevatorStructure.moveToStartingCommand());
        
        // Handoff algae
        operator.leftTrigger()
                .whileTrue(
                    elevatorStructure.handoffAlgaeCommand().until(() -> claw.hasAlgae())
                            .andThen(intake.setStateCommand(States.HANDOFFALGAE)
                            .andThen(Commands.waitSeconds(.1))
                            .andThen(elevatorStructure.pickupAlgaeCommand()))
                )
                .onFalse(Commands.waitSeconds(.04).andThen(intake.setStateCommand(States.DEFAULT)));

        // right bumper - barge
        operator.rightBumper().whileTrue(
            new ConditionalCommand(elevatorStructure.moveToBargeCommand(),
                    elevatorStructure.handoffAlgaeCommand().until(() -> claw.hasAlgae())
                            .andThen(intake.setStateCommand(States.HANDOFFALGAE)
                            .andThen(Commands.waitSeconds(.1))
                            .andThen(elevatorStructure.pickupAlgaeCommand())),() -> claw.hasAlgae())
                            )
            .onFalse(Commands.waitSeconds(.04).andThen(intake.setStateCommand(States.DEFAULT)));

        // Intake Coral
        operator.rightTrigger()
                .whileTrue(elevatorStructure.intakeCoralCommand().alongWith(drivetrain.applyRequest(() -> {
                    calculateMaxSpeed();
                    Rotation2d targetRotation = getRotationForIntake(drivetrain.getState().Pose.getRotation());
                    
                    return driveAtAngle.withVelocityX(-applyExpo(driver.getLeftY()) * CalculatedMaxSpeed) // Drive forward with
                                                                                               // negative Y (forward)
                            .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                            .withTargetDirection(targetRotation);
                }))).onFalse(elevatorStructure.moveToIntakeEndCommand());
        
        // Store algae                
        operator.leftBumper().onTrue(intake.setStateCommand(States.HANDOFFALGAE)
                .alongWith(elevatorStructure.storeAlgaeCommand()).andThen(intake.setStateCommand(States.DEFAULT)));

        // Start climb
        operator.back().and(operator.start()).onTrue(climb.setServo().alongWith(elevatorStructure.moveToClimb())
                .alongWith(intake.setStateCommand(States.CLIMBREADY)));

        // Align to joystick position
        operator.axisMagnitudeGreaterThan(0, .7).or(operator.axisMagnitudeGreaterThan(1, .7))
                .whileTrue(drivetrain.applyRequest(() -> {
                    calculateMaxSpeed();
                    Rotation2d targetRotation = getRotationForJoystick(operator.getLeftX(), -operator.getLeftY());

                    return driveAtAngle.withVelocityX(-applyExpo(driver.getLeftY()) * CalculatedMaxSpeed) // Drive forward with
                                                                                               // negative Y (forward)
                            .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                            .withTargetDirection(targetRotation);
                }));

        // Set reef direction
        //auxilary.button(1).whileTrue();
        //auxilary.button(2).whileTrue();
        //auxilary.button(3).whileTrue();
        //auxilary.button(4).whileTrue();
        //auxilary.button(5).whileTrue();
        //auxilary.button(6).whileTrue();

        // Climb Grab Cage
        driver.back().and(() -> RobotState.isClimbing).onTrue(intake.setStateCommand(States.GRABCAGE).alongWith(climb.moveToPosition(State.GRABCAGE)));
        // Climb Actually Climb
        driver.start().and(() -> RobotState.isClimbing)
                .onTrue(intake.setStateCommand(States.DISABLE).alongWith(climb.moveToPosition(State.ENDCLIMB)));

        // Floor Intake
        driver.leftTrigger().onTrue(intake.setStateCommand(States.FLOORINTAKE).alongWith(elevatorStructure.moveToFloorIntake()))
                .onFalse(intake.setStateCommand(States.DEFAULT).alongWith(elevatorStructure.moveToAlgaeStartingCommand()));
        // Floor Outtake
        driver.leftBumper().onTrue(intake.setStateCommand(States.OUTTAKE))
                .onFalse(intake.setStateCommand(States.DEFAULT));
        
        // Intake Algae based on position reef
        driver.a().and(() -> claw.hasAlgae()).onTrue(elevatorStructure.intakeAlgaeCommand());

        driver.y().whileTrue(vision.setPriorityIdCommand().andThen(driveToTarget(ReefSides.LEFT, false)).finallyDo(() -> vision.clearPriorityId())).onFalse(vision.clearPriorityIdCommand());
        driver.x().whileTrue(vision.setPriorityIdCommand().andThen(driveToTarget(ReefSides.RIGHT, false)).finallyDo(() -> vision.clearPriorityId())).onFalse(vision.clearPriorityIdCommand());
        driver.povDown().whileTrue(vision.setPriorityIdCommand().andThen(driveToTarget(ReefSides.CENTER, false)).finallyDo(() -> vision.clearPriorityId())).onFalse(vision.clearPriorityIdCommand());
        
        //driver.y().whileTrue(driveBackwardCommand());
        
        // Score Algage
        driver.rightTrigger().onTrue(Commands.either(elevatorStructure.scoreBargeCommand(), elevatorStructure.outtakeAlgaeCommand(), () -> elevator.getHeight() > 50))
            .onFalse(Commands.either(elevatorStructure.moveToEndBargeCommand(), elevatorStructure.moveToStartingCommand(), () -> elevator.getHeight() > 50));


        // Score Coral
        driver.rightBumper().and(() -> elevatorStructure.isAtPosition() && (!waitForPosition || isAtPosition(currentReefSide))).whileTrue(elevatorStructure.outtakeCoralCommand());

        // Align based on current angle to reef
        driver.b().whileTrue(drivetrain.applyRequest(() -> {
            calculateMaxSpeed();
            var rotation = Rotation2d.k180deg;
            if(getAllianceSide() != Robot.AllianceColor.get()) {
                rotation = Rotation2d.kZero;
            }

            var xVel = -applyExpo(driver.getLeftY()) * CalculatedMaxSpeed;
            if(elevator.getHeight() > ElevatorStructure.BargeRear.ElevatorHeight - 10 && arm.seeBarge()) {
                var xOutput = (arm.bargeError() / 100.0) * 4.0;
                var tmpXVel = MathUtil.clamp(xOutput, -1, 1);
            
                xVel = tmpXVel * (MaxSpeed / 5.0);
                if(arm.isAtBarge()) {
                    CommandScheduler.getInstance().schedule(elevatorStructure.scoreBargeCommand());
                }

                if(getAllianceSide() != Robot.AllianceColor.get()) {
                    xVel = -xVel;
                }
            }

            return driveAtAngle.withVelocityX(xVel) // Drive forward with
                                                                                       // negative Y (forward)
                    .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(rotation);
        }));

        driver.povUp().whileTrue(drivetrain.applyRequest(() -> {
            calculateMaxSpeed();
            var rotation = Rotation2d.kZero;
            if(getAllianceSide() != Robot.AllianceColor.get()) {
                rotation = Rotation2d.k180deg;
            }

            var xVel = -applyExpo(driver.getLeftY()) * CalculatedMaxSpeed;
            

            return driveAtAngle.withVelocityX(xVel) // Drive forward with
                                                                                       // negative Y (forward)
                    .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(rotation);
        }));


    }

    // public Rotation2d getRotationForBarge(double currentAngle) {
    // double reefSlice = (Math.PI * 2.0) / 6.0;
    // double reefSliceMiddle = reefSlice /2.0;
    // Rotation2d joystickRotation = new Rotation2d(requestX, requestY).rotateBy(new
    // Rotation2d((-Math.PI/2.0)-reefSliceMiddle));
    // SmartDashboard.putNumber("Drivetrain/joystickDegrees",
    // joystickRotation.getDegrees());

    // int reefIndex = (int)(joystickRotation.getRadians() / reefSlice);
    // if(Math.signum(joystickRotation.getRadians()) == 1) {
    // reefIndex++;
    // }

    // SmartDashboard.putNumber("Drivetrain/reefIndex", reefIndex);

    // Rotation2d output = new Rotation2d((reefIndex * reefSlice));
    // output = output.rotateBy(new Rotation2d(Math.PI));

    // SmartDashboard.putNumber("Drivetrain/reefDegrees", output.getDegrees());

    // return output;
    // }

    public Command moveToAlgaeLowCommand(){
        return elevatorStructure.moveToAlgaeLowCommand();
    }

    public Command moveToAlgaeHighCommand(){
        return elevatorStructure.moveToAlgaeHighCommand();
    }

    public int getPriorityId() {
        return priorityId;
    }
    
    public Command driveBackwardCommand(boolean right) {
        return drivetrain.applyRequest(() -> {
            var rotation = Rotation2d.fromDegrees(55);
            if(!right){
                rotation = Rotation2d.fromDegrees(-55);
            }
            return driveRobotCentric
            // TX = Front/Back
            .withVelocityX(-.15 * (MaxSpeed))
            .withTargetDirection(rotation)
            ;
        }
        ).until(() -> claw.hasCoral()).withTimeout(1);
    }

    private enum ReefSides { RIGHT, LEFT, CENTER }

    //public double getReefAngle(int targetId) {
//
//    }

    public boolean isAtPositionAuto(ReefSides side) {
        var errorPose = getError(side);
        if(Math.abs(errorPose.getX()) < .04 && Math.abs(errorPose.getY()) < .04 && Math.abs(errorPose.getRotation().getDegrees()) < 3){
            var speeds = drivetrain.getState().Speeds;
            if(Math.abs(speeds.vxMetersPerSecond) < .075 && Math.abs(speeds.vyMetersPerSecond) < .075 && Math.abs(speeds.omegaRadiansPerSecond) < .1){
                return true;
            }
            else 
                return false;
        }
        else{
            return false;
        }
    }

    public boolean isAtPosition(ReefSides side) {
        var errorPose = getError(side);
        if(Math.abs(errorPose.getX()) < .04 && Math.abs(errorPose.getY()) < .04 && Math.abs(errorPose.getRotation().getDegrees()) < 3){
            var speeds = drivetrain.getState().Speeds;
            if(Math.abs(speeds.vxMetersPerSecond) < .1 && Math.abs(speeds.vyMetersPerSecond) < .1 && Math.abs(speeds.omegaRadiansPerSecond) < .1){
                return true;
            }
            else 
                return false;
        }
        else{
            return false;
        }
    }

    public Pose2d getError(ReefSides side){

        // 7 R - .43, -.16
        //7 C - 
        //left
        
        
        // BLUE
        var goalX = Utils.getValue(.41, .42);
        var goalY = Utils.getValue(.16, .19);
        
        if(getAllianceSide() == Alliance.Red) {
            // RED
            goalX = Utils.getValue(.41, .43);
            goalY = Utils.getValue(.16, .21);
        }

        if(side == ReefSides.RIGHT) {
            goalX = Utils.getValue(.41, .42);
            goalY = Utils.getValue(-.11, -.13);    

            if(getAllianceSide() == Alliance.Red) {
                // RED
                goalX = Utils.getValue(.41, .43);
                goalY = Utils.getValue(.16, -.14);
            }    
        }
        else if(side == ReefSides.CENTER) {
            goalX = Utils.getValue(.39, .44);
            goalY = Utils.getValue(.09, .12);

            if(getAllianceSide() == Alliance.Red) {
                // RED
                goalX = Utils.getValue(.41, .43);
                goalY = Utils.getValue(.16, .12);
            }    
        }
        
        var xError = goalX - Math.abs(robotPoseInTargetSpace.getX());
        var yError = goalY - robotPoseInTargetSpace.getY();
        var robotRot = drivetrain.getState().Pose.getRotation();
        if(getAllianceSide() == Alliance.Red) {
            robotRot = robotRot.rotateBy(Rotation2d.k180deg);
        }

        var rotError = robotPoseInTargetSpace.getRotation().getDegrees() - robotRot.getDegrees();

        SmartDashboard.putNumber("Align/rotError", rotError);
        SmartDashboard.putNumber("Align/xError", xError);
        SmartDashboard.putNumber("Align/yError", yError);
        SignalLogger.writeDouble("Align/rotError", rotError);
        SignalLogger.writeDouble("Align/xError", xError);
        SignalLogger.writeDouble("Align/yError", yError);

        
        return new Pose2d(xError, yError, Rotation2d.fromDegrees(rotError));
    }

    public boolean closeToTarget(ReefSides side) {
        var error = getError(side);
        //var distance = Math.sqrt((error.getX()*error.getX())  + (error.getY()*error.getY()));
        return Math.abs(error.getX()) < .30;
    }

    public Command driveToTargetAuto(ReefSides side) {//, double angle) {
        return driveToTarget(side, true).until(() -> isAtPositionAuto(side)).andThen(drivetrain.applyRequestOnce(()->{
            return driveRobotCentric
            // TX = Front/Back
            .withVelocityX(0)
            // TY = Left/Right
            .withVelocityY(0)
            .withTargetDirection(robotPoseInTargetSpace.getRotation());
        })).withTimeout(1.5);
    }

    public Command driveToTarget(ReefSides side, boolean isAuto) {//, double angle) {
        // 82 X
        // 56.2 Y
        return drivetrain.applyRequest(() -> {
            if(robotPoseInTargetSpace.getX() == 0 && robotPoseInTargetSpace.getY() == 0 && robotPoseInTargetSpace.getRotation().getDegrees() == 0) {
                // If the robot pose is not initialized yet, don't try to align
                vision.clearPriorityId();
                if(!isAuto){
                    vision.setPriorityId();
                }
                return defaultDrive().get();
            }

             var targetAngle = robotPoseInTargetSpace.getRotation().getDegrees();
             var currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
          
            if(Robot.AllianceColor.get() == Alliance.Red) {
                currentAngle = drivetrain.getState().Pose.getRotation().rotateBy(Rotation2d.k180deg).getDegrees();
            }

            SmartDashboard.putNumber("Align/currentAngle", currentAngle);
            SignalLogger.writeDouble("Align/currentAngle", currentAngle);

            
            if(targetAngle == 0 && (currentAngle > 30 || currentAngle < -30)) {
                vision.clearPriorityId();
                if(!isAuto){
                    vision.setPriorityId();
                }
                return defaultDrive().get();    
            }
            else if(targetAngle == 180 && (currentAngle < 150 && currentAngle > -150)) {
                vision.clearPriorityId();
                if(!isAuto){
                    vision.setPriorityId();
                }
                return defaultDrive().get();    
            }
            else if(targetAngle != 0 && targetAngle != 180 && Math.abs(targetAngle - currentAngle) > 30) {
                vision.clearPriorityId();
                if(!isAuto){
                    vision.setPriorityId();
                }
                return defaultDrive().get();
            }
          
            currentReefSide = side;
            waitForPosition = true;
            
            // LEFT
            var errorPose = getError(side);

            var xOutput = errorPose.getX()* 4.0;
            var yOutput = errorPose.getY()* 4.0;

            double yVel = MathUtil.clamp(yOutput, -1, 1);
            double xVel = MathUtil.clamp(xOutput, -1, 1);
            
            var atPosition = isAtPosition(side);
            SmartDashboard.putBoolean("Align/atPosition", atPosition);
            SmartDashboard.putBoolean("Align/atPositionAuto", isAtPositionAuto(side));
            SmartDashboard.putNumber("Align/xVel", xVel);
            SmartDashboard.putNumber("Align/yVel", yVel);
            SignalLogger.writeBoolean("Align/atPosition", atPosition);
            SignalLogger.writeBoolean("Align/atPositionAuto", isAtPositionAuto(side));
            SignalLogger.writeDouble("Align/xVel", xVel);
            SignalLogger.writeDouble("Align/yVel", yVel);


            return driveRobotCentric
            // TX = Front/Back
            .withVelocityX(-xVel * (MaxSpeed/5.0))
            // TY = Left/Right
            .withVelocityY(yVel * (MaxSpeed/5.0))
            .withTargetDirection(robotPoseInTargetSpace.getRotation());
        }
        ).finallyDo(() -> waitForPosition = false);
    }

    // public Command driveToBarge(){
    //     return drivetrain.applyRequestOnce(()->{
    //         boolean atBarge = arm.isAtBarge();
    //         if(atBarge){
    //             return driveRobotCentric
    //             // TX = Front/Back
    //             .withVelocityX(0)
    //             // TY = Left/Right
    //             .withVelocityY(0)
    //             .withTargetDirection(robotPoseInTargetSpace.getRotation());
    //         }
    //         else{
    //             //temporary values. 
    //             return driveRobotCentric
    //             // TX = Front/Back
    //             .withVelocityX(0)
    //             // TY = Left/Right
    //             .withVelocityY(0)
    //             .withTargetDirection(robotPoseInTargetSpace.getRotation());
    //         }
            
    //     });
    // }

    double startWheelPosition;
    double endWheelPosition;
    double wheelRevolutions;
    double radiansOfRotation;
    double swerveTravel;
    double wheelCircumference;
    double wheelRadius;

    Rotation2d startRotationPosition;
    Rotation2d endRotationPosition;

    public Command rotationTestCommand() {
        return Commands.runOnce(() -> {
            startWheelPosition = drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
            startRotationPosition = robotPoseInTargetSpace.getRotation();    
        }, drivetrain)
        .andThen(drivetrain.applyRequest(() -> {
            return driveRobotCentric1
                .withRotationalRate(2.0);
        }
        )).withTimeout(4).andThen(Commands.runOnce(()-> {
            endWheelPosition = drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
            endRotationPosition = robotPoseInTargetSpace.getRotation();
            wheelRevolutions = endWheelPosition - startWheelPosition;
            radiansOfRotation = endRotationPosition.getRadians() - startRotationPosition.getRadians();
            swerveTravel = radiansOfRotation * (10.875 * .0254) * Math.sqrt(2.0);
            wheelCircumference = swerveTravel / wheelRevolutions;
            wheelRadius = wheelCircumference / (2 * Math.PI);
            
            SmartDashboard.putNumber("SwerveDrive/WheelSize", wheelRadius);
        }, drivetrain));
    }

    public Rotation2d getRotationForReef(Rotation2d currentRotation) {
        double reefSlice = (Math.PI * 2.0) / 6.0;
        double reefSliceMiddle = reefSlice / 2.0;

        double currentReefSlice;
        if (currentRotation.getRadians() > -reefSliceMiddle * 5.0 && currentRotation.getRadians() < reefSliceMiddle) {
            currentReefSlice = (int) (currentRotation.rotateBy(Rotation2d.fromRadians(-reefSliceMiddle)).getRadians()
                    / reefSlice);
        } else if (currentRotation.getRadians() < reefSliceMiddle * 5.0
                && Math.signum(currentRotation.getRadians()) == 1) {
            currentReefSlice = (int) (currentRotation.rotateBy(Rotation2d.fromRadians(reefSliceMiddle)).getRadians()
                    / reefSlice);
        } else {
            currentReefSlice = 3;
        }

        Rotation2d joystickRotation = Rotation2d.fromRadians((reefSlice * currentReefSlice));
        // SmartDashboard.putNumber("Drivetrain/joystickDegrees",
        // joystickRotation.getDegrees());
        // SmartDashboard.putNumber("Drivetrain/joystickDegrees1",
        // (currentRotation.rotateBy(Rotation2d.fromRadians(-reefSliceMiddle)).getDegrees()));

        if (Robot.AllianceColor.get() == Alliance.Red) {
            joystickRotation = joystickRotation.rotateBy(Rotation2d.fromDegrees(180));
        }

        return joystickRotation;
    }

    public Rotation2d getRotationForIntake(Rotation2d currentRotation) {
        var intakeRotation = Rotation2d.fromDegrees(55);
        var invertCheck = Robot.AllianceColor.get() == Alliance.Blue ? -1 : 1;

        if (Math.signum(currentRotation.getRadians()) == invertCheck) {
            intakeRotation = Rotation2d.fromDegrees(intakeRotation.getDegrees() * -1);
        }

        return intakeRotation;
    }

    public Rotation2d getRotationForJoystick(double requestX, double requestY) {
        double reefSlice = (Math.PI * 2.0) / 6.0;
        double reefSliceMiddle = reefSlice / 2.0;
        Rotation2d joystickRotation = new Rotation2d(requestX, requestY)
                .rotateBy(new Rotation2d((-Math.PI / 2.0) - reefSliceMiddle));
        // SmartDashboard.putNumber("Drivetrain/joystickDegrees",
        // joystickRotation.getDegrees());

        int reefIndex = (int) (joystickRotation.getRadians() / reefSlice);
        if (Math.signum(joystickRotation.getRadians()) == 1) {
            reefIndex++;
        }

        // SmartDashboard.putNumber("Drivetrain/reefIndex", reefIndex);

        Rotation2d output = new Rotation2d((reefIndex * reefSlice));
        output = output.rotateBy(new Rotation2d(Math.PI));

        // SmartDashboard.putNumber("Drivetrain/reefDegrees", output.getDegrees());

        return output;
    }

    public void updateTargetPose(Pose2d robotPoseInTargetSpace) {
        this.robotPoseInTargetSpace = robotPoseInTargetSpace;

        SmartDashboard.putNumberArray("Align/Target", new Double[] { robotPoseInTargetSpace.getX(), robotPoseInTargetSpace.getY(), robotPoseInTargetSpace.getRotation().getDegrees() });
        SignalLogger.writeDoubleArray("Align/Target", new double[] { robotPoseInTargetSpace.getX(), robotPoseInTargetSpace.getY(), robotPoseInTargetSpace.getRotation().getDegrees() });
    }

    public void updateReefPose(Pose2d pose) {

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private boolean hasSetOrientation = false;

    public void updateAlliance(Alliance allianceColor) {
        vision.setIMUMode(0);
        if(!hasSetOrientation) {
            hasSetOrientation = true;
        }
        vision.setAprilTagFilter(VisionConstant.allReefTagIds);
    }

    public Alliance getAllianceSide() {
        if(drivetrain.getState().Pose.getX() > 8.79) {
            return Alliance.Red;
        }
        else {
            return Alliance.Blue;
        }
    }

    public Supplier<SwerveRequest> defaultDrive() {
                      // Drivetrain will execute this command periodically
                      return () -> {
                        if (RobotState.isClimbing) {
                            return driveAtAngle.withVelocityX(-applyExpo(driver.getLeftY()) * CalculatedMaxSpeed) // Drive forward with
                                                                                                       // negative Y
                                                                                                       // (forward)
                                    .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X
                                                                                            // (left)
                                    .withTargetDirection(Rotation2d.kCCW_90deg);
                        }
    
                        calculateMaxSpeed();
    
                        return drive.withVelocityX(-applyExpo(driver.getLeftY()) * CalculatedMaxSpeed) // Drive forward with negative Y
                                                                                            // (forward)
                                .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-applyExpo(driver.getRightX()) * CalculatedMaxAngularRate); // Drive
                                                                                                     // counterclockwise
                                                                                                     // with negative X
                                                                                                     // (left)
                    };
    }
}
