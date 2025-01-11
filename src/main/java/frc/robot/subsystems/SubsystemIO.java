package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemIO extends SubsystemBase {
    public void writeToLog() {}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();
}
