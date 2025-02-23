package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import java.util.Optional;
import limelight.Limelight;
import limelight.estimator.*;
import limelight.structures.*;
import limelight.structures.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {
  private final Limelight limelight = new Limelight("limelight-april");

  private LimelightPoseEstimator poseEstimator;
  private LimelightTargetData limelightTargetData;

  public LimelightSubsystem() {
    limelight
        .settingsBuilder()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(FieldConstants.Offsets.CAMERA_OFFSET)
        .save();
    poseEstimator = limelight.getPoseEstimator(true);
    limelightTargetData = new LimelightTargetData(limelight);
  }

  public Optional<PoseEstimate> getVisionEstimate() {
    return poseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
  }

  public int getID() {
    return (int) limelightTargetData.getAprilTagID();
  }

  public void updateSettings(Orientation3d orientation3d) {
    limelight.settingsBuilder().withRobotOrientation(orientation3d).save();
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
