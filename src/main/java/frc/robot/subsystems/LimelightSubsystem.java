package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import limelight.Limelight;
import limelight.estimator.*;
import limelight.structures.*;
import limelight.structures.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {
  private final Limelight limelight = new Limelight("limelight");

  public LimelightSubsystem() {
    limelight
        .getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(Pose3d.kZero)
        .save();
  }

  public boolean hasTarget() {
    LimelightTargetData limelighttargetdata = new LimelightTargetData(limelight);
    return limelighttargetdata.getTargetStatus();
  }

  public Pose2d getPosition() {
    PoseEstimator poseEstimator = new PoseEstimator<>(null, null, null, null);
    Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(true).getPoseEstimate();
    // If the pose is present
    visionEstimate.ifPresent(
        (PoseEstimate poseEstimate) -> {
          // Add it to the pose estimator.
          poseEstimator.addVisionMeasurement(
              poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
        });
    return poseEstimator.getEstimatedPosition();
  }

  public int getID() {
    LimelightTargetData limelighttargetdata = new LimelightTargetData(limelight);
    return (int) limelighttargetdata.getAprilTagID();
  }

  public void updateSettings(Orientation3d orientation3d) {
    limelight.getSettings().withRobotOrientation(orientation3d).save();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Sees Apriltag", hasTarget());
    SmartDashboard.putNumber("AprilTag ID", getID());
  }
}
