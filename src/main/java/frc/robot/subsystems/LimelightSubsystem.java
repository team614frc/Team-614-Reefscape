package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import java.util.Optional;
import limelight.Limelight;
import limelight.estimator.*;
import limelight.structures.*;
import limelight.structures.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {
  private final Limelight limelightfront = new Limelight("limelight-april");
  private final Limelight limelightback = new Limelight("limelight-coral");

  private LimelightPoseEstimator poseEstimator;
  private LimelightTargetData limelightTargetDataApril;
  private LimelightTargetData limelightTargetDataCoral;

  public LimelightSubsystem() {
    limelightfront
        .settingsBuilder()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(FieldConstants.Offsets.CAMERA_OFFSET)
        .save();
    poseEstimator = limelightfront.getPoseEstimator(true);
    limelightTargetDataApril = new LimelightTargetData(limelightfront);
    limelightback
    .settingsBuilder()
    .withLimelightLEDMode(LEDMode.PipelineControl)
    .withCameraOffset(FieldConstants.Offsets.CAMERA_OFFSET)
    .save();
    limelightTargetDataCoral = new LimelightTargetData(limelightback);
  }

  public Optional<PoseEstimate> getVisionEstimate() {
    return poseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
  }

  public int getID() {
    return (int) limelightTargetDataApril.getAprilTagID();
  }

  public void updateSettings(Orientation3d orientation3d) {
    limelightfront.settingsBuilder().withRobotOrientation(orientation3d).save();
    limelightback.settingsBuilder().withRobotOrientation(orientation3d).save();
  }

  public boolean hasTargetApril() {
    return limelightTargetDataApril.getTargetStatus();
  }

  public boolean hasTargetCoral() {
    return limelightTargetDataCoral.getTargetStatus();
  }

  public Pose2d getTargetCoral() {
    return limelightTargetDataCoral.getTargetToRobot().toPose2d();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Sees AprilTag", hasTargetApril());
    SmartDashboard.putBoolean("Sees Coral", hasTargetCoral());
    SmartDashboard.putNumber("AprilTag ID", getID());
  }
}
