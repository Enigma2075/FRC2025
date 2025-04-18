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
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.VisionLL.PoseObservation;
import frc.robot.subsystems.VisionLL.PoseObservationType;
import frc.robot.subsystems.VisionLL.VisionIOInputs;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utils;

public class Vision extends SubsystemIO {
  private final VisionConsumer consumer;
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<Alliance> allianceSupplier;
  private final TargetPoseConsumer targetPoseConsumer;
  private final TargetConsumer targetConsumer;

  private int priorityId = -1;

  private int currentTargetId = -1;

  private final VisionLL[] io;
  private final VisionLL.VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, Supplier<Rotation2d> rotationSupplier, Supplier<Alliance> allianceSupplier, TargetPoseConsumer targetPoseConsumer,
      TargetConsumer targetConsumer) {
    this.consumer = consumer;
    this.rotationSupplier = rotationSupplier;
    this.allianceSupplier = allianceSupplier;
    this.targetPoseConsumer = targetPoseConsumer;
    this.targetConsumer = targetConsumer;
    this.io = new VisionLL[] {
        new VisionLL(VisionConstant.kFrontRightLLName, rotationSupplier, this::getPriorityId),
        new VisionLL(VisionConstant.kFrontLeftLLName, rotationSupplier, this::getPriorityId),
        new VisionLL(VisionConstant.kBackRightLLName, rotationSupplier, this::getPriorityId),
        new VisionLL(VisionConstant.kBackLeftLLName, rotationSupplier, this::getPriorityId),
        new VisionLL(VisionConstant.kFrontMiddleLLName, rotationSupplier, this::getPriorityId)
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

  public void setAprilTagFilter(double[] aprilTagFilter){
    for (int i = 0; i < io.length; i++) {
      io[i].setAprilTagFilter(aprilTagFilter);
    }
  }

  public void setIMUMode(int mode){
    for (int i = 0; i < io.length; i++) {
      io[i].setIMUMode(mode);
    }
  }

  public int getPriorityId() {
    return priorityId;
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

  @FunctionalInterface
  public static interface TargetConsumer {
    public void accept(
        Pose2d RobotPoseInTargetSpace);
  }

  @Override
  public void readPeriodicInputs() {


    boolean sentTarget = false;
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);

      var logName = "Vision/" + Integer.toString(i);
      SmartDashboard.putBoolean(logName + "Connected", inputs[i].connected);
      SmartDashboard.putNumber(logName + "LatestObservation-tx", inputs[i].latestTargetObservation.tx().getRadians());
      SmartDashboard.putNumber(logName + "LatestObservation-ty", inputs[i].latestTargetObservation.ty().getRadians());
      SmartDashboard.putNumber(logName + "TargetId", inputs[i].targetId);
      
      SignalLogger.writeBoolean(logName + "Connected", inputs[i].connected);
      SignalLogger.writeDouble(logName + "LatestObservation-tx", inputs[i].latestTargetObservation.tx().getRadians());
      SignalLogger.writeDouble(logName + "LatestObservation-ty", inputs[i].latestTargetObservation.ty().getRadians());
      SignalLogger.writeDouble(logName + "TargetId", inputs[i].targetId);
      
      
      for (int j = 0; j < inputs[i].poseObservations.length ; j++) {
        var observation = inputs[i].poseObservations[j].pose().toPose2d();
        SmartDashboard.putNumberArray(logName + "PoseObservation" + Integer.toString(j), new double [] {observation.getX(), observation.getY(), observation.getRotation().getRadians()});
        SignalLogger.writeDoubleArray(logName + "PoseObservation" + Integer.toString(j), new double [] {observation.getX(), observation.getY(), observation.getRotation().getRadians()});
      }

      if(inputs[i].targetId != -1 && !sentTarget) {
        //sentTarget = true;
        //targetConsumer.accept(inputs[i].latestTargetObservation.tx().getRadians(), inputs[i].latestTargetObservation.ty().getRadians());
      }

      // (logName + "LatestObservation-ty", inputs[i].poseObservations);

      // Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    if(!sentTarget) {
      //targetConsumer.accept(0, 0);
    }

    List<VisionIOInputs> reefInputs = new LinkedList<>();
    for (int i = 0; i < 2 && i < inputs.length; i++) {
      double[] reefTags = VisionConstant.blueReefTagIds;
      if(allianceSupplier.get() == Alliance.Red) {
        reefTags = VisionConstant.redReefTagIds;
      }
    
      for (var tagId : reefTags) {
        if (tagId == inputs[i].targetId) {
          reefInputs.add(inputs[i]);
       }
      }
    }

    SmartDashboard.putNumber("Vision/ReefPoseCount", reefInputs.size());
    SignalLogger.writeInteger("Vision/ReefPoseCount", reefInputs.size());

    if(reefInputs.size() > 0 && targetPoseConsumer != null) {
      if(reefInputs.size() > 1) {
        SmartDashboard.putNumber("Vision/ReefPoseCount", reefInputs.size());
        SignalLogger.writeInteger("Vision/ReefPoseCount", reefInputs.size());
      }

      var reefInput = reefInputs.get(0);
      var reefPose = reefInput.targetPose;
      var reefPose2d = reefPose.toPose2d();
      SmartDashboard.putNumberArray("Vision/ReefPose", new double [] {reefPose2d.getX(), reefPose2d.getY(), reefPose2d.getRotation().getRadians()});
      SignalLogger.writeDoubleArray("Vision/ReefPose", new double [] {reefPose2d.getX(), reefPose2d.getY(), reefPose2d.getRotation().getRadians()});
      
      var targetTags = VisionConstant.blueTagTargets;
      var rotateAngle = false;
      if (allianceSupplier.get() == Alliance.Red) {
        if(Robot.AllianceColor.get() != Alliance.Red) {
          rotateAngle = true;
        }
        targetTags = VisionConstant.redTagTargets;
      }
      else {
        if(Robot.AllianceColor.get() != Alliance.Blue) {
          rotateAngle = true;
        }
      }

      SmartDashboard.putNumber("Vision/priorityID", priorityId);
      SignalLogger.writeInteger("Vision/priorityID", priorityId);

      double degrees = Double.MIN_VALUE;
      for (var tag : targetTags) {
        if (tag.id() == reefInput.targetId && (tag.id() == priorityId || priorityId == -1)) {
          degrees = tag.degrees();
          if(rotateAngle) {
            degrees = Rotation2d.fromDegrees(degrees).rotateBy(Rotation2d.k180deg).getDegrees();
          }
          targetConsumer.accept(new Pose2d(reefPose.getZ(), reefPose.getX(), Rotation2d.fromDegrees(degrees)));
          break;
        }
      }

      if(degrees == Double.MIN_VALUE) {
        targetConsumer.accept(new Pose2d());
      }
      
      currentTargetId = (int)reefInput.targetId;
      SmartDashboard.putNumber("Vision/currentTargetID", currentTargetId);
      SignalLogger.writeInteger("Vision/currentTargetID", currentTargetId);

      targetPoseConsumer.accept(reefPose2d);
    }else {
      targetPoseConsumer.accept(new Pose2d());
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

  public Command setPriorityId(int id) {
    return runOnce(() -> priorityId = id);
  }

  public Command setPriorityId(int blueId, int redId) {
    return runOnce(() -> {
      if (Robot.AllianceColor.get() == Alliance.Blue) {
        priorityId = blueId;
      } else {
        priorityId = redId;
      }
    });
  }

  public Command setPriorityIdCommand() {
    return runOnce(() -> {
      setPriorityId();
    });
  }

  public void setPriorityId() {
    priorityId = currentTargetId;
  }

  public Command clearPriorityIdCommand() {
    return runOnce(() -> {
      clearPriorityId();
    });
  }

  public void clearPriorityId() {
    
      priorityId = -1;
  }
}
