package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;

public class VisionLL {
    private final String m_name;

    private boolean m_connected = false;
    private TargetObservation m_latestTargetObservation = null;
    
    public VisionLL(String name) {
        m_name = name;
        var table = NetworkTableInstance.getDefault().getTable(name);
        //txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        //tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    }

      /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    public void update() {
        // get if the LL has been updated in the last 250 ms
        m_connected = (Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline(m_name)) / 1000 < 250;

        // m_latestTargetObservation = new TargetObservation(
        //     Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees()
        // );

    }


}
