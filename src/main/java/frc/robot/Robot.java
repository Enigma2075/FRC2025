// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RobotConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer RobotContainer;

  public Robot() {
    RobotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    RobotContainer.ioManager.readPeriodicInputs();
    RobotContainer.ioManager.outputTelemetry();
    
    CommandScheduler.getInstance().run(); 
    
    RobotContainer.ioManager.writePeriodicOutputs();
  }

  @Override
  public void disabledInit() {
    RobotContainer.ioManager.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = RobotContainer.getAutonomousCommand();

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
