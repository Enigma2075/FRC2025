// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorConst;
import frc.robot.subsystems.ElevatorStructure;
import frc.robot.subsystems.IOManager;
import frc.robot.subsystems.IntakeConstants;
import frc.robot.subsystems.RobotConstants;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristConstants;
import frc.robot.subsystems.states.ElevatorStructurePosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ClimbConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeConstants;
import frc.robot.subsystems.DriveTrainConstants;

public class RobotContainer {
    private double MaxSpeed = DriveTrainConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final Drivetrain drivetrain = DriveTrainConstants.createDrivetrain();

    public final Elevator elevator = new Elevator();

    public final Climb climb = new Climb();

    public final Arm arm = new Arm();

    //public final Claw claw = new Claw();

    public final Wrist wrist = new Wrist();
    
    public final ElevatorStructure elevatorStructure = new ElevatorStructure(elevator, arm);

    public final Intake intake = new Intake();

    private SendableChooser<Command> autoChooser; 

    public IOManager ioManager; 

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Test");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("PracticeBot", RobotConstants.kPracticeBot);
        
        configureBindings();

        //autoChooser = new SendableChooser<>();

        //autoChooser.addOption("Test", drivetrain.getAutoPath("Test")); 

        ioManager = new IOManager(climb, elevator, arm, wrist, intake, elevatorStructure);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Test Code for each subsystem
        //elevator.setDefaultCommand(elevator.testCommand(driver::getRightTriggerAxis));

        //climb.setDefaultCommand(climb.testCommand(() -> {return -operator.getLeftY();}));

        //arm.setDefaultCommand(arm.testCommand(() -> {return -operator.getLeftY();}));

        //operator.y().whileTrue(elevatorStructure.moveToL4(false));
        //operator.a().whileTrue(elevatorStructure.moveToStarting());
        //operator.b().whileTrue(elevatorStructure.moveToIntakeCoral(false));
        operator.x().whileTrue(elevatorStructure.moveToClimb());

        //operator.leftBumper().whileTrue(arm.setTestPosition(90));
        //operator.rightBumper().whileTrue(elevator.setTestPosition(30));
        //operator.a().whileTrue(arm.setTestPosition(10));
        
        //operator.a().whileTrue(intake.sysIdQuasiStatic(Direction.kReverse));
        //operator.b().whileTrue(intake.sysIdDynamic(Direction.kReverse));
        //operator.x().whileTrue(intake.sysIdDynamic(Direction.kForward));
        //operator.y().whileTrue(intake.sysIdQuasiStatic(Direction.kForward));

        //operator.y().whileTrue(climb.setTestPosition());
        /*
        operator.b().whileTrue(arm.setTestPosition());
        operator.a().whileTrue(claw.setTestPosition());
        operator.x().whileTrue(wrist.setTestPosition());
        operator.y().whileTrue(intake.setTestPosition());
        */

        //operator.a().whileTrue(elevator.setTestPosition(1));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
