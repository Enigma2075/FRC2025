package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.VisionLL.PoseObservationType;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class Vision extends SubsystemIO {
  private final VisionConsumer consumer;
  private final Supplier<Rotation2d> rotationSupplier;
  private final TargetPoseConsumer targetPoseConsumer;

  private final VisionLL[] io;
  private final VisionLL.VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, Supplier<Rotation2d> rotationSupplier, TargetPoseConsumer targetPoseConsumer) {
    this.consumer = consumer;
    this.rotationSupplier = rotationSupplier;
    this.targetPoseConsumer = targetPoseConsumer;
    this.io = new VisionLL[] {
        new VisionLL(VisionConstant.kFrontRightLLName, rotationSupplier),
        new VisionLL(VisionConstant.kFrontLeftLLName, rotationSupplier),
        new VisionLL(VisionConstant.kBackRightLLName, rotationSupplier),
        new VisionLL(VisionConstant.kBackLeftLLName, rotationSupplier)
    };

    // Initialize inputs
    this.inputs = new VisionLL.VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionLL.VisionIOInputs();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing
   * with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  public static interface TargetPoseConsumer {
    public void accept(
        Pose2d targetPose3d);
  }

  @Override
  public void readPeriodicInputs() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);

      var logName = "Vision/" + Integer.toString(i);
      SmartDashboard.putBoolean(logName + "Connected", inputs[i].connected);
      SmartDashboard.putNumber(logName + "LatestObservation-tx", inputs[i].latestTargetObservation.tx().getRadians());
      SmartDashboard.putNumber(logName + "LatestObservation-ty", inputs[i].latestTargetObservation.ty().getRadians());
      
      SignalLogger.writeBoolean(logName + "Connected", inputs[i].connected);
      SignalLogger.writeDouble(logName + "LatestObservation-tx", inputs[i].latestTargetObservation.tx().getRadians());
      SignalLogger.writeDouble(logName + "LatestObservation-ty", inputs[i].latestTargetObservation.ty().getRadians());
      
      // (logName + "LatestObservation-ty", inputs[i].poseObservations);

      // Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    List<Pose3d> reefTagPoses = new LinkedList<>();
    for (int i = 0; i < 2; i++) {
      int[] reefTags = VisionConstant.blueReefTagIds;
      if(Robot.AllianceColor.get() == Alliance.Red) {
        reefTags = VisionConstant.redReefTagIds;
      }
    
      for (var tagId : reefTags) {
        if (tagId == inputs[i].targetId) {
          reefTagPoses.add(inputs[i].targetPose);
        }
      }
    }

    if(reefTagPoses.size() > 0 && targetPoseConsumer != null) {
      var reefPose = reefTagPoses.get(0).toPose2d();
      SmartDashboard.putNumberArray("Vision/ReefPose", new double [] {reefPose.getX(), reefPose.getY(), reefPose.getRotation().getRadians()});
      SignalLogger.writeDoubleArray("Vision/ReefPose", new double [] {reefPose.getX(), reefPose.getY(), reefPose.getRotation().getRadians()});
      targetPoseConsumer.accept(reefPose);
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
        boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
            || (observation.tagCount() == 1
                && observation.ambiguity() > VisionConstant.maxAmbiguity) // Cannot be high ambiguity
            || Math.abs(observation.pose().getZ()) > VisionConstant.maxZError // Must have realistic Z coordinate

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
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
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

        var pose1 = observation.pose().toPose2d();
        SmartDashboard.putNumberArray("Vision/Pose", new double [] {pose1.getX(), pose1.getY(), pose1.getRotation().getRadians()});
        SignalLogger.writeDoubleArray("Vision/Pose", new double [] {pose1.getX(), pose1.getY(), pose1.getRotation().getRadians()});
        SmartDashboard.putNumberArray("Vision/Dev", new double [] {linearStdDev, angularStdDev});
        SignalLogger.writeDoubleArray("Vision/Dev", new double [] {linearStdDev, angularStdDev});
        
        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      // Logger.recordOutput(
      // "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
      // tagPoses.toArray(new Pose3d[tagPoses.size()]));
      // Logger.recordOutput(
      // "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
      // robotPoses.toArray(new Pose3d[robotPoses.size()]));
      // Logger.recordOutput(
      // "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
      // robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      // Logger.recordOutput(
      // "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
      // robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    // Logger.recordOutput(
    // "Vision/Summary/TagPoses", allTagPoses.toArray(new
    // Pose3d[allTagPoses.size()]));
    // Logger.recordOutput(
    // "Vision/Summary/RobotPoses", allRobotPoses.toArray(new
    // Pose3d[allRobotPoses.size()]));
    // Logger.recordOutput(
    // "Vision/Summary/RobotPosesAccepted",
    // allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    // Logger.recordOutput(
    // "Vision/Summary/RobotPosesRejected",
    // allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method
    // 'outputTelemetry'");
  }

}
