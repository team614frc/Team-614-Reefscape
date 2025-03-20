// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  private LimelightSubsystem limelightFront;

  private LimelightSubsystem limelightBack;

  private Field2d field = new Field2d();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being
    // created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(
                  Constants.MAX_SPEED.in(MetersPerSecond),
                  new Pose2d(
                      new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
      if (Constants.DrivebaseConstants.USE_LIMELIGHT_FRONT)
        limelightFront = new LimelightSubsystem(Constants.DrivebaseConstants.LIMELIGHT_FRONT_NAME);
      if (Constants.DrivebaseConstants.USE_LIMELIGHT_BACK)
        limelightBack = new LimelightSubsystem(Constants.DrivebaseConstants.LIMELIGHT_BACK_NAME);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
    // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(
        true, true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(
        false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    swerveDrive.setChassisDiscretization(true, 0.02);
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(
      SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive =
        new SwerveDrive(
            driveCfg,
            controllerCfg,
            Constants.MAX_SPEED.in(MetersPerSecond),
            new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  @Override
  public void simulationPeriodic() {}

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive
              // trains
              // Translation PID constants
              new PIDConstants(
                  DrivebaseConstants.AUTO_TRANSLATION_kP,
                  DrivebaseConstants.AUTO_TRANSLATION_kI,
                  DrivebaseConstants.AUTO_TRANSLATION_kD),
              // Rotation PID constants
              new PIDConstants(
                  DrivebaseConstants.AUTO_ROTATION_kP,
                  DrivebaseConstants.AUTO_ROTATION_kI,
                  DrivebaseConstants.AUTO_ROTATION_kD)),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
          );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command printCurrentPose() {
    return Commands.none(); // dCommands.deferredProxy(()->Commands.print("Current Pose:
    // "+getPose().toString()));
  }

  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            Constants.DrivebaseConstants.MAX_ALIGNMENT_VELOCITY,
                Constants.DrivebaseConstants.MAX_ALIGNMENT_ACCELERATION,
            Constants.DrivebaseConstants.MAX_ALIGNMENT_ANGULAR_VELOCITY,
                Constants.DrivebaseConstants.MAX_ALIGNMENT_ANGULAR_ACCELERATION);
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    double tooCloseMeters =
        0.5; // If the bot is too close by this much it needs to drive back a little bit.
    return defer(() -> driveToPose(pose.get()));
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * <p>If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    // We changed to !isRedAlliance after testing because blue and red are inverted when run with
    // the boolean isRedAlliance()
    if (!isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED.in(MetersPerSecond));
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Orientation3d getOrientation3d() {
    return (new Orientation3d(
        swerveDrive.getGyroRotation3d(),
        new AngularVelocity3d(
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(0),
            swerveDrive.getGyro().getYawAngularVelocity())));
  }

  public void addVisionReading(PoseEstimate pose) {
    swerveDrive.addVisionMeasurement(pose.pose.toPose2d(), pose.timestampSeconds);
  }
  
  @Override
  public void periodic() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    if (Constants.DrivebaseConstants.USE_LIMELIGHT_FRONT) {
      limelightFront.updateSettings(getOrientation3d());
      updatePosition(limelightFront.getVisionEstimate());
    }
    if (Constants.DrivebaseConstants.USE_LIMELIGHT_BACK) {
      limelightBack.updateSettings(getOrientation3d());
      updatePosition(limelightBack.getVisionEstimate());
    }
    // SmartDashboard.putNumber("Closest AprilTagID", findReefID());
    SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot X Coordinates", getPose().getX());
    SmartDashboard.putNumber("Robot Y Coordinates", getPose().getY());
    // SmartDashboard.putBoolean("Field-Centric", isFieldCentric);
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
  }

  public void updatePosition(Optional<PoseEstimate> visionEstimate) {
    visionEstimate.ifPresent(
        (PoseEstimate poseEstimate) -> {
          // If the average tag distance is less than 4 meters,
          // there are more than 0 tags in view,
          // and the average ambiguity between tags is less than 30% then we update the pose
          // estimation.
          SmartDashboard.putNumber("Est X", poseEstimate.pose.toPose2d().getX());
          SmartDashboard.putNumber("Est Y", poseEstimate.pose.toPose2d().getY());
          if (poseEstimate.avgTagDist < 4
              && poseEstimate.tagCount > 0
              && poseEstimate.getMinTagAmbiguity() < 0.3) {
            addVisionReading(poseEstimate);
          }
        });
  }

  // public Pose2d getTarget(Direction direction) {
  //   Pose2d path = new Pose2d();
  //   Optional<Alliance> ally = DriverStation.getAlliance();
  //   if (ally.isEmpty() || ally.get() == Alliance.Red) {
  //     path =
  // swerveDrive.getPose().nearest(FieldConstants.Reef.BRANCH_POSITIONS_RED.get(direction));
  //   } else {
  //     path =
  //
  // swerveDrive.getPose().nearest(FieldConstants.Reef.BRANCH_POSITIONS_BLUE.get(direction));
  //   }
  //   return path;
  // }

  public boolean isFieldCentric = true;

  public Command flipFieldAndRobotRelative() {
    return Commands.runOnce(() -> isFieldCentric = !isFieldCentric, this);
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}
