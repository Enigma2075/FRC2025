package frc.robot.subsystems;

import frc.robot.util.Utils;

public class VisionConstant {

    public static final String kFrontRightLLName = Utils.getValue("limelight-dwayne", "limelight-billie");
    public static final String kFrontLeftLLName = Utils.getValue("limelight-terry", "limelight-sabrina");
    public static final String kBackRightLLName = Utils.getValue("limelight-john", "limelight-sza");
    public static final String kBackLeftLLName = Utils.getValue("limelight-kevin", "limelight-nicki");

    public static final Vision Vision = new Vision();
}
