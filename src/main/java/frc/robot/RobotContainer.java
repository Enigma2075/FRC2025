// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.Wrist;
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
    private final RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final Drivetrain drivetrain = DriveTrainConstants.createDrivetrain();

    public final Elevator elevator = new Elevator();

    public final Climb climb = new Climb();

    public final Arm arm = new Arm();

    public final Claw claw = new Claw();

    public final Wrist wrist = new Wrist();

    public final Intake intake = new Intake();
    
    public final ElevatorStructure elevatorStructure = new ElevatorStructure(elevator, arm, wrist, claw, (state) -> intake.setStateCommand(state));
    
    public final Vision vision = new Vision(drivetrain::addVisionMeasurement,
            () -> drivetrain.getState().Pose.getRotation(), this::updateReefPose, this::updateTarget);

    private int priorityId = 0;

    private double tx = 0;
    private double ty = 0;

    private SendableChooser<Command> autoChooser;

    public IOManager ioManager;

    public RobotContainer() {
        //drivetrain.setStateStdDevs(VecBuilder.fill(9999, 9999, 0));
        driveAtAngle.HeadingController.setP(10);
        driveAtAngle.MaxAbsRotationalRate = MaxAngularRate;

        NamedCommands.registerCommand("intake", elevatorStructure.intakeCoralCommand());
        NamedCommands.registerCommand("drive_forward", elevatorStructure.intakeCoralCommand().alongWith(driveBackwardCommand()).until(() -> claw.hasCoral()).withTimeout(1));
        NamedCommands.registerCommand("move_to_L4", elevatorStructure.moveToL4Command());
        NamedCommands.registerCommand("outtake1", vision.setPriorityId(22, 9).alongWith(driveToTarget(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand())));
        NamedCommands.registerCommand("outtake2", vision.setPriorityId(17, 8).alongWith(driveToTarget(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand())));
        NamedCommands.registerCommand("outtake3", vision.setPriorityId(20, 11).alongWith(driveToTarget(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand())));
        NamedCommands.registerCommand("outtake4", vision.setPriorityId(19, 6).alongWith(driveToTarget(ReefSides.LEFT).andThen(elevatorStructure.autoOuttakeCoralCommand())));


        ioManager = new IOManager(climb, elevator, arm, wrist, claw, intake, elevatorStructure, vision);

        SmartDashboard.putBoolean("PracticeBot", RobotConstants.kPracticeBot);

        configureBindings();

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Test", drivetrain.getAutoPath("Test"));
        //autoChooser.addOption("Test", drivetrain.getAutoPath("Test"));

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
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
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
                }));

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
                .onTrue(
                    elevatorStructure.handoffAlgaeCommand().until(() -> claw.hasAlgae())
                                .andThen(intake.setStateCommand(States.HANDOFFALGAE)
                                        .andThen(elevatorStructure.pickupAlgaeCommand()))
                                        .andThen(intake.setStateCommand(States.DEFAULT))
                );

        // Intake Coral
        operator.rightTrigger()
                .whileTrue(elevatorStructure.intakeCoralCommand().alongWith(drivetrain.applyRequest(() -> {
                    calculateMaxSpeed();
                    Rotation2d targetRotation = getRotationForIntake(drivetrain.getState().Pose.getRotation());
                    
                    return driveAtAngle.withVelocityX(-applyExpo(driver.getLeftY()) * CalculatedMaxSpeed) // Drive forward with
                                                                                               // negative Y (forward)
                            .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                            .withTargetDirection(targetRotation);
                }))).onFalse(elevatorStructure.moveToStartingCommand());

        // right bumper - barge
        operator.rightBumper().whileTrue(
                new ConditionalCommand(elevatorStructure.moveToBargeCommand(),
                        elevatorStructure.handoffAlgaeCommand().until(() -> claw.hasAlgae())
                                .andThen(intake.setStateCommand(States.HANDOFFALGAE)
                                        .andThen(elevatorStructure.pickupAlgaeCommand())),
                        () -> claw.hasAlgae()).andThen(intake.setStateCommand(States.DEFAULT)));
        
        // Store algae                
        operator.leftBumper().onTrue(intake.setStateCommand(States.HANDOFFALGAE)
                .alongWith(elevatorStructure.storeAlgaeCommand()).andThen(intake.setStateCommand(States.DEFAULT)));

        // Start climb
        operator.back().onTrue(climb.setServo().alongWith(elevatorStructure.moveToClimb())
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

        // Climb Grab Cage
        driver.back().and(() -> RobotState.isClimbing).onTrue(intake.setStateCommand(States.GRABCAGE));
        // Climb Actually Climb
        driver.start().and(() -> RobotState.isClimbing)
                .onTrue(intake.setStateCommand(States.DISABLE).alongWith(climb.moveToPosition(State.ENDCLIMB)));

        // Floor Intake
        driver.leftTrigger().onTrue(intake.setStateCommand(States.FLOORINTAKE))
                .onFalse(intake.setStateCommand(States.DEFAULT));
        // Floor Outtake
        driver.leftBumper().onTrue(intake.setStateCommand(States.OUTTAKE))
                .onFalse(intake.setStateCommand(States.DEFAULT));
        
        // Intake Algae based on position
        driver.a().onTrue(elevatorStructure.intakeAlgaeCommand());

        driver.y().whileTrue(vision.setPriorityId().alongWith(driveToTarget(ReefSides.LEFT)));
        driver.x().whileTrue(vision.setPriorityId().alongWith(driveToTarget(ReefSides.RIGHT)));
        // Score Algage
        driver.rightTrigger().onTrue(elevatorStructure.outtakeAlgaeCommand());

        // Score Coral
        driver.rightBumper().whileTrue(elevatorStructure.outtakeCoralCommand());

        // Align based on current angle to reef
        driver.b().whileTrue(drivetrain.applyRequest(() -> {
            calculateMaxSpeed();
            Rotation2d targetRotation = getRotationForReef(drivetrain.getState().Pose.getRotation());

            return driveAtAngle.withVelocityX(-applyExpo(driver.getLeftY()) * CalculatedMaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
                    .withVelocityY(-applyExpo(driver.getLeftX()) * CalculatedMaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(targetRotation);
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

    public int getPriorityId() {
        return priorityId;
    }
    
    public Command driveBackwardCommand() {
        return drivetrain.applyRequest(() -> {
            return driveRobotCentric
            // TX = Front/Back
            .withVelocityX(-.25 * (MaxSpeed))
            ;
        }
        ).withTimeout(1);
    }

    private enum ReefSides { RIGHT, LEFT }

    public Command driveToTarget(ReefSides side) {
        // Offset Target = .15, -.21
        

        // 82 X
        // 56.2 Y
        return drivetrain.applyRequest(() -> {
            // LEFT
            var goalX = .38;
            var goalY = .145;
            if(side == ReefSides.RIGHT) {
                goalX = .38;
                goalY = -.145;    
            }
            
            var xError = goalX - tx;
            var yError = goalY - ty;

            xError *= 2.0;
            yError *= 6.0;

            double yVel = MathUtil.clamp(yError, -1, 1);
            double xVel = MathUtil.clamp(xError, -1, 1);
            
            SmartDashboard.putNumber("Align/xVel", xVel);
            SmartDashboard.putNumber("Align/yVel", yVel);
            SignalLogger.writeDouble("Align/xVel", xVel);
            SignalLogger.writeDouble("Align/yVel", yVel);

            return driveRobotCentric
            // TX = Front/Back
            .withVelocityX(-xVel * (MaxSpeed/6.0))
            // TY = Left/Right
            .withVelocityY(yVel * (MaxSpeed/6.0))
            ;
        }
        ).withTimeout(1.5);
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

    public void updateTarget(double tx, double ty) {
        SmartDashboard.putNumberArray("Align/Target", new Double[] { tx, ty });
        this.tx = tx;
        this.ty = ty;
    }

    public void updateReefPose(Pose2d pose) {

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
