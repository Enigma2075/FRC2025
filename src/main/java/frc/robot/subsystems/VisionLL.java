package frc.robot.subsystems;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LimelightHelpers;

public class VisionLL {
    public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public long targetId = 0;
        public Pose3d targetPose = new Pose3d();
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    private final String name;

    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;

    private final Supplier<Rotation2d> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;
    private final IntegerPublisher priorityIdPublisher;
    private final DoubleSubscriber latencySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;
    private final DoubleArraySubscriber targetPoseSubscriber;
    private final IntegerSubscriber targetIdSubscriber;
    private final DoubleArrayPublisher fiducialIdFiltersSetPublisher;
    private final IntegerPublisher imuModePublisher;

    private final Supplier<Integer> priorityIdSupplier;

    public VisionLL(String name, Supplier<Rotation2d> rotationSupplier, Supplier<Integer> targetIdSupplier) {
        this.name = name;
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotationSupplier = rotationSupplier;
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

        targetPoseSubscriber = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
        targetIdSubscriber = table.getIntegerTopic("tid").subscribe(0);

        priorityIdPublisher = table.getIntegerTopic("priorityid").publish();

        fiducialIdFiltersSetPublisher = table.getDoubleArrayTopic("fiducial_id_filters_set").publish();

        imuModePublisher = table.getIntegerTopic("imu_mode").publish();

        this.priorityIdSupplier = targetIdSupplier;
    }

    public void setAprilTagFilter(double[] aprilTagFilter){
        fiducialIdFiltersSetPublisher.accept(aprilTagFilter);
    }

    public void setIMUMode(int mode){
        imuModePublisher.accept(mode);
    }

    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last
        // 250ms
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        if(!inputs.connected) {
            return;
        }

        //var priorityId = priorityIdSupplier.get();
        //if(priorityId != -1) {
        //    priorityIdPublisher.accept(priorityIdSupplier.get());
        //}

        inputs.targetId = targetIdSubscriber.get();
        inputs.targetPose = parsePose(targetPoseSubscriber.get());
        
        // Update target observation
        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

        // Update orientation for MegaTag 2
        orientationPublisher.accept(
                new double[] { rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });
        NetworkTableInstance.getDefault()
                .flush(); // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        // for (var rawSample : megatag1Subscriber.readQueue()) {
        //     if (rawSample.value.length == 0)
        //         continue;
        //     for (int i = 11; i < rawSample.value.length; i += 7) {
        //         tagIds.add((int) rawSample.value[i]);
        //     }
        //     poseObservations.add(
        //             new PoseObservation(
        //                     // Timestamp, based on server timestamp of publish and latency
        //                     rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

        //                     // 3D pose estimate
        //                     parsePose(rawSample.value),

        //                     // Ambiguity, using only the first tag because ambiguity isn't applicable for
        //                     // multitag
        //                     rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

        //                     // Tag count
        //                     (int) rawSample.value[7],

        //                     // Average tag distance
        //                     rawSample.value[9],

        //                     // Observation type
        //                     PoseObservationType.MEGATAG_1));
        // }
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0 || rawSample.value.length < 10)
                continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(
                    new PoseObservation(
                            // Timestamp, based on server timestamp of publish and latency
                            rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                            // 3D pose estimate
                            parsePose(rawSample.value),

                            // Ambiguity, zeroed because the pose is already disambiguated
                            0.0,

                            // Tag count
                            (int) rawSample.value[7],

                            // Average tag distance
                            rawSample.value[9],

                            // Observation type
                            PoseObservationType.MEGATAG_2));
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        if(rawLLArray.length < 6) {
            return new Pose3d();
        }

        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }
}
