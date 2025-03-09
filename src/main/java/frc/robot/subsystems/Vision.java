package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionLL.PoseObservationType;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class Vision extends SubsystemIO {
  private final VisionConsumer consumer;
  private final VisionLL[] io;
  private final VisionLL.VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionLL... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionLL.VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionLL.VisionIOInputs();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      
      SmartDashboard.putBoolean("Vision/"+Integer.toString(i), io[i].connected);
      //SmartDashboard.putBoolean("Vision/"+Integer.toString(i), io[i].);

      //Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = VisionConstant.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > VisionConstant.maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConstant.maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > VisionConstant.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > VisionConstant.aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstant.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstant.angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= VisionConstant.linearStdDevMegatag2Factor;
          angularStdDev *= VisionConstant.angularStdDevMegatag2Factor;
        }
        if (cameraIndex < VisionConstant.cameraStdDevFactors.length) {
          linearStdDev *= VisionConstant.cameraStdDevFactors[cameraIndex];
          angularStdDev *= VisionConstant.cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
    //       tagPoses.toArray(new Pose3d[tagPoses.size()]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
    //       robotPoses.toArray(new Pose3d[robotPoses.size()]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
    //       robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
    //       robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    // Logger.recordOutput(
    //     "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    // Logger.recordOutput(
    //     "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    // Logger.recordOutput(
    //     "Vision/Summary/RobotPosesAccepted",
    //     allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    // Logger.recordOutput(
    //     "Vision/Summary/RobotPosesRejected",
    //     allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

@Override
public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
}

@Override
public boolean checkSystem() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
}

@Override
public void outputTelemetry() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
}

}
