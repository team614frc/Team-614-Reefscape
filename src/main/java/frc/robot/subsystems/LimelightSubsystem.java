package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.List;
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
        .withPipelineIndex(0)
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(Constants.CAMERA_OFFSET)
        .withArilTagIdFilter(List.of(17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0))
        .save();

    poseEstimator = limelight.getPoseEstimator(true);
    limelightTargetData = new LimelightTargetData(limelight);
  }

  public Optional<PoseEstimate> getVisionEstimate() {
    return poseEstimator.getPoseEstimate();
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
