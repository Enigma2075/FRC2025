// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public boolean hasAutoRun = false;

  public static Optional<Alliance> AllianceColor = null;
  private static boolean hasAlliance = false;

  public static RobotContainer RobotContainer;

  private double disabledTimer = Double.MAX_VALUE;

  public Robot() {
    RobotContainer = new RobotContainer();
    //RobotContainer.logger.stop();
  }

  @Override
  public void robotInit() {
    //RobotContainer.logger.stop();
  }

  @Override
  public void robotPeriodic() {
    if (!hasAlliance || DriverStation.isDisabled()) {
      AllianceColor = DriverStation.getAlliance();
      AllianceColor.ifPresent((allianceColor) -> {
        SmartDashboard.putString("Robot/Alliance", allianceColor.name());
        hasAlliance = true;
        RobotContainer.updateAlliance(allianceColor);
      });

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

    // set a timer so we know how long we have been disabled
    disabledTimer = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    // stop the logger after 5 seconds of being disabled
    if (Timer.getFPGATimestamp() - disabledTimer > 10) {
      //RobotContainer.logger.stop();
      disabledTimer = Double.MAX_VALUE;
    }

    if(!hasAutoRun && hasAlliance) {
      var autoCommand = (PathPlannerAuto) RobotContainer.getAutonomousCommand();
      var pose = autoCommand.getStartingPose();
      
      if(AllianceColor.get() == Alliance.Red){
        pose = FlippingUtil.flipFieldPose(pose);
      }

      RobotContainer.drivetrain.resetPose(pose);
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    //RobotContainer.logger.stop();
    //RobotContainer.logger.start();
    RobotContainer.elevatorStructure.applyAutoStartPosition();
    m_autonomousCommand = RobotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      hasAutoRun = true;
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    //RobotContainer.logger.start();
    RobotContainer.elevator.setOverrideVelocity(false);

    RobotContainer.vision.clearPriorityId();
    RobotContainer.elevatorStructure.clearPress();

    if (hasAutoRun == false) {
        hasAutoRun = true;
      if (AllianceColor.get() == Alliance.Blue) {
        RobotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(0));
      }
      else {
        RobotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(180));
      }
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
