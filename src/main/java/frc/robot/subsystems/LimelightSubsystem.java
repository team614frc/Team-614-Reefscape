package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.DetectionMode;
import limelight.Limelight;
import limelight.structures.*;
import limelight.structures.LimelightSettings.LEDMode;

public class LimelightSubsystem extends SubsystemBase {
  // private final Limelight limelightFront = new Limelight("limelight-front");
  private final LimelightPipelineData limelightPipelineDataBack;
  private final Limelight limelightBack = new Limelight("limelight-back");

  // private LimelightPoseEstimator poseEstimator;
  // private LimelightTargetData limelightTargetDataApril;
  private LimelightTargetData limelightTargetDataBack;

  public LimelightSubsystem() {
    // limelightfront
    //     .settingsBuilder()
    //     .withLimelightLEDMode(LEDMode.PipelineControl)
    //     .withCameraOffset(FieldConstants.Offsets.CAMERA_OFFSET)
    //     .save();
    // poseEstimator = limelightfront.getPoseEstimator(true);
    // limelightTargetDataApril = new LimelightTargetData(limelightfront);
    limelightBack
        .settingsBuilder()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(FieldConstants.Offsets.CAMERA_OFFSET)
        .save();
    limelightTargetDataBack = new LimelightTargetData(limelightBack);
    limelightPipelineDataBack = new LimelightPipelineData(limelightBack);
  }

  // public Optional<PoseEstimate> getVisionEstimate() {
  //   return poseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
  // }

  // public int getID() {
  //   return (int) limelightTargetDataApril.getAprilTagID();
  // }

  public void updateSettings(Orientation3d orientation3d) {
    // limelightfront.settingsBuilder().withRobotOrientation(orientation3d).save();
    limelightBack.settingsBuilder().withRobotOrientation(orientation3d).save();
  }

  // public boolean hasTargetApril() {
  //   return limelightTargetDataApril.getTargetStatus();
  // }

  public void setMode(DetectionMode mode) {
    int detectionmode = (mode == FieldConstants.DetectionMode.CORAL) ? 1 : 0;
    limelightBack.settingsBuilder().withPipelineIndex(detectionmode);
  }

  public Angle getTargetAngleBack() {
    return Degrees.of(limelightTargetDataBack.getHorizontalOffset());
  }

  public DetectionMode getMode() {
    return (limelightPipelineDataBack.getCurrentPipelineIndex() == 0)
        ? FieldConstants.DetectionMode.CORAL
        : FieldConstants.DetectionMode.APRILTAG;
  }

  public boolean hasTargetBack() {
    return (limelightTargetDataBack.getTargetStatus());
  }

  public Pose2d getTargetBack() {
    return limelightTargetDataBack.getTargetToRobot().toPose2d();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Sees Target Front", hasTargetApril());
    SmartDashboard.putBoolean("Sees Target Back", hasTargetBack());
    // SmartDashboard.putNumber("AprilTag ID", getID());
    SmartDashboard.putNumber("Back Angle", getTargetAngleBack().in(Degrees));
  }
}
