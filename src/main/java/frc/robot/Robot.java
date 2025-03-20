// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorStructure;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public boolean hasAutoRun = false;
  public boolean hasSetOdometry = false;
  
  public static Optional<Alliance> AllianceColor = null;
  private static boolean hasAlliance = false;

  public static RobotContainer RobotContainer;

  public Robot() {
    RobotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    if(!hasAlliance || DriverStation.isDisabled()) {
      if (!hasAlliance || DriverStation.isDisabled()) {
        AllianceColor = DriverStation.getAlliance();
        AllianceColor.ifPresent((allianceColor) -> {
          SmartDashboard.putString("Robot/Alliance", allianceColor.name());
          hasAlliance = true;
          RobotContainer.updateAlliance(allianceColor);
        });
      }
    }

    RobotContainer.ioManager.readPeriodicInputs();
    RobotContainer.ioManager.outputTelemetry();

    RobotState.outputTelemetry();
    
    CommandScheduler.getInstance().run(); 
    
    RobotContainer.ioManager.writePeriodicOutputs();
  }

  @Override
  public void disabledInit() {
    RobotContainer.ioManager.stop();
    RobotContainer.logger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    RobotContainer.logger.start();
    RobotContainer.elevatorStructure.applyAutoStartPosition();
    m_autonomousCommand = RobotContainer.getAutonomousCommand();
    hasAutoRun = true;
    hasSetOdometry = true;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    RobotContainer.logger.start();
    if(!hasSetOdometry) {
      if(AllianceColor.isPresent() && AllianceColor.get() == Alliance.Red) {
        RobotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(180));
        hasSetOdometry = true;
      }
      else {
        RobotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(0));
        hasSetOdometry = true;
      }
    }

    if (hasAutoRun == false) {
      //odometry to specific place
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
