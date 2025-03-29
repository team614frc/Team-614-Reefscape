package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants.DetectionMode;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.*;
import limelight.networktables.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {
  private Limelight limelight;

  private LimelightPoseEstimator poseEstimator;
  private LimelightTargetData limelightTargetData;

  public LimelightSubsystem(String name) {
    limelight = new Limelight(name);
    limelight
        .getSettings()
        .withPipelineIndex(DetectionMode.APRILTAG.ordinal())
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(Constants.CAMERA_OFFSET)
        .withArilTagIdFilter(Constants.DrivebaseConstants.APRIL_TAGS)
        .save();

    poseEstimator = limelight.getPoseEstimator(true);
    limelightTargetData = new LimelightTargetData(limelight);
  }

  public Optional<PoseEstimate> getVisionEstimate() {
    return poseEstimator.getPoseEstimate();
  }

  public void setPipeline(DetectionMode mode) {
    limelight.getSettings().withPipelineIndex(mode.ordinal());
  }

  public Optional<LimelightResults> getResults() {
    return limelight.getLatestResults();
  }

  public int getID() {
    return (int) limelightTargetData.getAprilTagID();
  }

  public void updateSettings(Orientation3d orientation3d) {
    limelight.getSettings().withRobotOrientation(orientation3d).save();
  }

  public boolean hasTarget() {
    return limelightTargetData.getTargetStatus();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Sees AprilTag", hasTarget());
    SmartDashboard.putNumber("AprilTag ID", getID());
  }
}
