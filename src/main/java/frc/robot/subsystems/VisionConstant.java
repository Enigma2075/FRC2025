package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.VisionLL.PoseObservationType;
import frc.robot.util.Utils;

public class VisionConstant {

        /** Represents a robot pose sample used for pose estimation. */
        public static record TagTarget(
                        double id,
                        double degrees) {
        }

        public static final String kFrontRightLLName = Utils.getValue("limelight-dwayne", "limelight-billie");
        public static final String kFrontLeftLLName = Utils.getValue("limelight-terry", "limelight-sabrina");
        public static final String kBackRightLLName = Utils.getValue("limelight-john", "limelight-sza");
        public static final String kBackLeftLLName = Utils.getValue("limelight-kevin", "limelight-nicki");

        // Robot to camera transforms
        // (Not used by Limelight, configure in web UI instead)
        public static Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
        public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.02;
        public static double maxZError = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = .02; // Meters
        public static double angularStdDevBaseline = 10; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] cameraStdDevFactors = new double[] {
                        1.0, // Camera 0
                        1.0, // Camera 1
                        1.0, // Camera 2
                        1.0 // Camera 3
        };

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor = 9999999; // No rotation data available

        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2025ReefscapeWelded);

        public static TagTarget[] blueTagTargets = new TagTarget[] {
                        new TagTarget(17, 60.0),
                        new TagTarget(18, 0.0),
                        new TagTarget(19, -60.0),
                        new TagTarget(20, -120.0),
                        new TagTarget(21, 180.0),
                        new TagTarget(22, 120.0)
        };

        public static TagTarget[] redTagTargets = new TagTarget[] {
                        new TagTarget(6, -60.0),
                        new TagTarget(7, 0.0),
                        new TagTarget(8, 60.0),
                        new TagTarget(9, 120.0),
                        new TagTarget(10, 180.0),
                        new TagTarget(11, -1200.0)
        };

        public static double[] blueReefTagIds = java.util.Arrays.stream(blueTagTargets)
                        .mapToDouble(TagTarget::id)
                        .toArray();

        public static double[] redReefTagIds = java.util.Arrays.stream(redTagTargets)
                        .mapToDouble(TagTarget::id)
                        .toArray();
}
