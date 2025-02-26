package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    public enum ScoringSides { FRONT, BACK }

    public static ScoringSides scoringSide = ScoringSides.FRONT;

    public static void outputTelemetry(){
        SmartDashboard.putBoolean("RobotState/ScoringFront", scoringSide==ScoringSides.FRONT);

        SignalLogger.writeBoolean("RobotState/ScoringFront", scoringSide==ScoringSides.FRONT);
    }
}
