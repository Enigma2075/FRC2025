package frc.robot.subsystems;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class Vision extends SubsystemIO {
    public Vision() {
        LimelightHelpers.setPipelineIndex(VisionConstant.kFrontRightLLName, 0);
        LimelightHelpers.setPipelineIndex(VisionConstant.kFrontLeftLLName, 0);
        LimelightHelpers.setPipelineIndex(VisionConstant.kBackRightLLName, 0);
        LimelightHelpers.setPipelineIndex(VisionConstant.kBackLeftLLName, 0);
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

    

    public void frontLeftOuttake(){
        
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
