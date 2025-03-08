package frc.robot.subsystems;

import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemIO {
    public Vision() {
        LimelightHelpers.setPipelineIndex(getName(), 0);
    }

    public double getHorizAngleFromTag(String limelightName) {
        double targetAngle = 0.0;
        String tempLimelightName = limelightName;

        if (LimelightHelpers.getTV(tempLimelightName) == true) {
            targetAngle = LimelightHelpers.getTXNC(tempLimelightName);
            return targetAngle;
        } else {
            return 0.0;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'readPeriodicInputs'");
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'readPeriodicOutputs'");
    }
}
