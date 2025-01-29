package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import limelight.Limelight;
import limelight.estimator.*;
import limelight.structures.*;
import limelight.structures.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {
  private final Limelight limelight;
  LimelightPoseEstimator poseEstimator;
  LimelightTargetData limelighttargetdata;

  public LimelightSubsystem() {
    limelight = new Limelight("limelight-april");
    limelight
        .settingsBuilder()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(VisionConstants.cameraOffset)
        .save();
    poseEstimator = limelight.getPoseEstimator(true);
    limelighttargetdata = new LimelightTargetData(limelight);
  }

  public Optional<PoseEstimate> getVisionEstimate() {
    return poseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
  }

  public Pose2d getVisionEstimateTest() {
    return limelighttargetdata.getTargetToRobot().toPose2d();
  }

  public int getID() {
    return (int) limelighttargetdata.getAprilTagID();
  }

  public void updateSettings(Orientation3d orientation3d) {
    limelight.settingsBuilder().withRobotOrientation(orientation3d).save();
  }

  public boolean hasTarget() {
    return limelighttargetdata.getTargetStatus();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Sees AprilTag", hasTarget());
    SmartDashboard.putNumber("AprilTag ID", getID());
  }
}
