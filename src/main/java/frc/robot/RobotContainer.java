// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private double CalculatedMaxSpeed = MaxSpeed;
    private double CalculatedMaxAngularRate = MaxAngularRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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

    public final ElevatorStructure elevatorStructure = new ElevatorStructure(elevator, arm, wrist, claw);

    public final Intake intake = new Intake();

    public final Vision vision = new Vision(drivetrain::addVisionMeasurement, () -> drivetrain.getState().Pose.getRotation(), null);

    private SendableChooser<Command> autoChooser;

    public IOManager ioManager;

    public RobotContainer() {
        driveAtAngle.HeadingController.setP(10);
        driveAtAngle.MaxAbsRotationalRate = MaxAngularRate;

        NamedCommands.registerCommand("intake", elevatorStructure.intakeCoralCommand().until(() -> claw.hasCoral()));
        NamedCommands.registerCommand("move_to_L4", elevatorStructure.moveToL4Command());
        NamedCommands.registerCommand("outtake", elevatorStructure.outtakeCoralCommand().until(() -> !claw.hasCoral()));

        ioManager = new IOManager(climb, elevator, arm, wrist, claw, intake, elevatorStructure, vision);

        SmartDashboard.putBoolean("PracticeBot", RobotConstants.kPracticeBot);

        configureBindings();

        autoChooser = new SendableChooser<>();

        autoChooser.addOption("Test", drivetrain.getAutoPath("Test"));

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

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    if (RobotState.isClimbing) {
                        return driveAtAngle.withVelocityX(-driver.getLeftY() * CalculatedMaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                                .withVelocityY(-driver.getLeftX() * CalculatedMaxSpeed) // Drive left with negative X
                                                                                        // (left)
                                .withTargetDirection(Rotation2d.kCCW_90deg);
                    }

                    calculateMaxSpeed();

                    return drive.withVelocityX(-driver.getLeftY() * CalculatedMaxSpeed) // Drive forward with negative Y
                                                                                        // (forward)
                            .withVelocityY(-driver.getLeftX() * CalculatedMaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(-driver.getRightX() * CalculatedMaxAngularRate); // Drive
                                                                                                 // counterclockwise
                                                                                                 // with negative X
                                                                                                 // (left)
                }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
        //     RobotState.scoringSide = ScoringSides.BACK;
        // }));
        // operator.povLeft().whileTrue(Commands.run(() -> {
        //     RobotState.scoringSide = ScoringSides.FRONT;
        // }));

        operator.povUp().whileTrue(elevatorStructure.moveToAlgaeHighCommand())
                .onFalse(elevatorStructure.clearAlgaePress());
        operator.povDown().whileTrue(elevatorStructure.moveToAlgaeLowCommand())
                .onFalse(elevatorStructure.clearAlgaePress());

        operator.y().whileTrue(elevatorStructure.moveToL4Command()).onFalse(elevatorStructure.clearCoralPress());
        operator.b().whileTrue(elevatorStructure.moveToL3Command()).onFalse(elevatorStructure.clearCoralPress());
        operator.a().whileTrue(elevatorStructure.moveToL2Command()).onFalse(elevatorStructure.clearCoralPress());
        operator.x().whileTrue(elevatorStructure.moveToL1Command()).onFalse(elevatorStructure.clearCoralPress());

        operator.rightStick().whileTrue(elevatorStructure.moveToStartingCommand());
        operator.leftTrigger()
                .onTrue(intake.setStateCommand(States.HANDOFFALGAE).alongWith(elevatorStructure.handoffAlgaeCommand()));

        operator.rightTrigger()
                .whileTrue(elevatorStructure.intakeCoralCommand().alongWith(drivetrain.applyRequest(() -> {
                    calculateMaxSpeed();
                    Rotation2d targetRotation = getRotationForIntake(drivetrain.getState().Pose.getRotation());
                    //Rotation2d targetRotation = getRotationForReef(drivetrain.getState().Pose.getRotation());
                    
                    return driveAtAngle.withVelocityX(-driver.getLeftY() * CalculatedMaxSpeed) // Drive forward with
                                                                                               // negative Y (forward)
                            .withVelocityY(-driver.getLeftX() * CalculatedMaxSpeed) // Drive left with negative X (left)
                            .withTargetDirection(targetRotation);
                }))).onFalse(elevatorStructure.moveToStartingCommand());

        // right bumper - barge
        operator.rightBumper().whileTrue(
                new ConditionalCommand(elevatorStructure.moveToBargeCommand(),
                        elevatorStructure.handoffAlgaeCommand().until(() -> claw.hasAlgae())
                                .andThen(intake.setStateCommand(States.HANDOFFALGAE)
                                        .andThen(elevatorStructure.pickupAlgaeCommand())),
                        () -> claw.hasAlgae()).andThen(intake.setStateCommand(States.DEFAULT)));
        operator.leftBumper().onTrue(intake.setStateCommand(States.HANDOFFALGAE)
                .alongWith(elevatorStructure.storeAlgaeCommand()).andThen(intake.setStateCommand(States.DEFAULT)));

        operator.back().onTrue(climb.setServo().alongWith(elevatorStructure.moveToClimb())
                .alongWith(intake.setStateCommand(States.CLIMBREADY)));

        operator.axisMagnitudeGreaterThan(0, .7).or(operator.axisMagnitudeGreaterThan(1, .7))
                .whileTrue(drivetrain.applyRequest(() -> {
                    calculateMaxSpeed();
                    Rotation2d targetRotation = getRotationForJoystick(operator.getLeftX(), -operator.getLeftY());
                    
                    return driveAtAngle.withVelocityX(-driver.getLeftY() * CalculatedMaxSpeed) // Drive forward with
                                                                                               // negative Y (forward)
                            .withVelocityY(-driver.getLeftX() * CalculatedMaxSpeed) // Drive left with negative X (left)
                            .withTargetDirection(targetRotation);
                }));

        driver.start().and(() -> RobotState.isClimbing).onTrue(intake.setStateCommand(States.GRABCAGE));
        driver.back().and(() -> RobotState.isClimbing)
                .onTrue(intake.setStateCommand(States.DISABLE).alongWith(climb.moveToPosition(State.ENDCLIMB)));

        driver.leftTrigger().onTrue(intake.setStateCommand(States.FLOORINTAKE))
                .onFalse(intake.setStateCommand(States.DEFAULT));
        driver.rightTrigger().onTrue(elevatorStructure.outtakeAlgaeCommand());

        driver.a().onTrue(elevatorStructure.intakeAlgaeCommand());

        driver.leftBumper().onTrue(intake.setStateCommand(States.OUTTAKE))
                .onFalse(intake.setStateCommand(States.DEFAULT));
        driver.rightBumper().whileTrue(elevatorStructure.outtakeCoralCommand());


        driver.b().whileTrue(drivetrain.applyRequest(() -> {
            calculateMaxSpeed();
            Rotation2d targetRotation = getRotationForReef(drivetrain.getState().Pose.getRotation());
            
            return driveAtAngle.withVelocityX(-driver.getLeftY() * CalculatedMaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * CalculatedMaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(targetRotation);
        }));
    }

    // public void setDriveAtAngleP(Rotation2d targetRotation) {
    //     double error = targetRotation.minus(drivetrain.getState().Pose.getRotation()).getRadians();

    //     SmartDashboard.putNumber("Drive/AngleError", error);

    //     if (elevator.getHeight() > 20) {
    //         if (Math.abs(error) < Math.PI / 8.0) {
    //             driveAtAngle.HeadingController.setP(10);
    //         } else if (Math.abs(error) < Math.PI / 2.0) {
    //             driveAtAngle.HeadingController.setP(2.5);
    //         } else {
    //             driveAtAngle.HeadingController.setP(1.5);
    //         }
    //     } else {
    //         driveAtAngle.HeadingController.setP(10);
    //     }
    // }

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

    public Rotation2d getRotationForReef(Rotation2d currentRotation) {
        double reefSlice = (Math.PI * 2.0) / 6.0;
        double reefSliceMiddle = reefSlice / 2.0;

        double currentReefSlice;
        if(currentRotation.getRadians() > -reefSliceMiddle * 5.0 && currentRotation.getRadians() < reefSliceMiddle) {
            currentReefSlice = (int)(currentRotation.rotateBy(Rotation2d.fromRadians(-reefSliceMiddle)).getRadians() / reefSlice);
        }
        else if(currentRotation.getRadians() < reefSliceMiddle * 5.0 && Math.signum(currentRotation.getRadians() ) ==1) {
            currentReefSlice = (int)(currentRotation.rotateBy(Rotation2d.fromRadians(reefSliceMiddle)).getRadians() / reefSlice);
        }
        else {
            currentReefSlice = 3;
        }

        Rotation2d joystickRotation = Rotation2d.fromRadians((reefSlice * currentReefSlice));
        //SmartDashboard.putNumber("Drivetrain/joystickDegrees", joystickRotation.getDegrees());
        //SmartDashboard.putNumber("Drivetrain/joystickDegrees1", (currentRotation.rotateBy(Rotation2d.fromRadians(-reefSliceMiddle)).getDegrees()));
        
        if(Robot.AllianceColor.get() == Alliance.Red) {
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
        //SmartDashboard.putNumber("Drivetrain/joystickDegrees", joystickRotation.getDegrees());

        int reefIndex = (int) (joystickRotation.getRadians() / reefSlice);
        if (Math.signum(joystickRotation.getRadians()) == 1) {
            reefIndex++;
        }

        //SmartDashboard.putNumber("Drivetrain/reefIndex", reefIndex);

        Rotation2d output = new Rotation2d((reefIndex * reefSlice));
        output = output.rotateBy(new Rotation2d(Math.PI));

        //SmartDashboard.putNumber("Drivetrain/reefDegrees", output.getDegrees());

        return output;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
