package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IOManager extends SubsystemBase {
      // The robot's subsystems and commands are defined here...
  private List<SubsystemIO> m_AllSubsystems = new ArrayList<>();

    public IOManager(SubsystemIO... allSubsystems) {
        m_AllSubsystems = Arrays.asList(allSubsystems);
    }

    @Override
    public void periodic() {
    }

    public Command defaultCommand() {
        return run(() -> {
        });
    }

    public void readPeriodicInputs() {
        m_AllSubsystems.forEach(SubsystemIO::readPeriodicInputs);
      }
    
      public void writePeriodicOutputs() {
        m_AllSubsystems.forEach(SubsystemIO::writePeriodicOutputs);
      }

      public void stop() {
        m_AllSubsystems.forEach(SubsystemIO::stop);
      }
    
      public void outputTelemetry() {
        m_AllSubsystems.forEach(SubsystemIO::outputTelemetry);
      }
    
}
