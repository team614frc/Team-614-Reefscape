// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController codriverXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  SwerveInputStream driveAngularVelocitySim =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocitySim =
      drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveReef(boolean isleft) {
    return Commands.runOnce(
        () -> {
          String chosenpath;
          if (LimelightHelpers.getTV(null)) // checks to see if there is valid apriltag to target
          {
            switch ((int)
                LimelightHelpers.getFiducialID(
                    null)) { // chooses path based on which april tag id is detected and value of
                // isleft
              case 6 -> chosenpath = (isleft) ? "Right6Path" : "Right6Path";

              case 7 -> chosenpath = (isleft) ? "Right7Path" : "Right7Path";

              case 8 -> chosenpath = (isleft) ? "Left6Path" : "Right6Path";

              case 9 -> chosenpath = (isleft) ? "Left8Path" : "Right8Path";

              case 10 -> chosenpath = (isleft) ? "Left10Path" : "Right10Path";

              case 11 -> chosenpath = (isleft) ? "Left11Path" : "Right11Path";

              case 17 -> chosenpath = (isleft) ? "Left17Path" : "Right17Path";

              case 18 -> chosenpath = (isleft) ? "Left18Path" : "Right18Path";

              case 19 -> chosenpath = (isleft) ? "Left19Path" : "Right19Path";

              case 20 -> chosenpath = (isleft) ? "Left20Path" : "Rightt20Path";

              case 21 -> chosenpath = (isleft) ? "Left21Path" : "Right21Path";

              case 22 -> chosenpath = (isleft) ? "Left22Path" : "Right22Path";
              default -> {}
            }
            /* PathPlannerPath path = PathPlannerPath.fromPathFile(chosenpath);
                          PathConstraints constraints = new PathConstraints(3.0, 4.0,
                          Units.degreesToRadians(540), Units.degreesToRadians(720));

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints); */
          }
        },
        drivebase);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverXbox.a().onTrue(Commands.none());
    driverXbox.x().onTrue(Commands.none());
    driverXbox.b().onTrue(Commands.none());
    driverXbox.y().onTrue(Commands.none());
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(Commands.none());
    driverXbox.leftBumper().onTrue(Commands.none());
    driverXbox.rightBumper().onTrue(Commands.none());

    codriverXbox.a().onTrue(Commands.none());
    codriverXbox.x().onTrue(Commands.none());
    codriverXbox.b().onTrue(Commands.none());
    codriverXbox.y().onTrue(Commands.none());
    codriverXbox.start().onTrue(Commands.none());
    codriverXbox.back().onTrue(Commands.none());
    codriverXbox.leftBumper().onTrue(Commands.none());
    codriverXbox.rightBumper().onTrue(Commands.none());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedAnglularVelocity
            : driveFieldOrientedAngularVelocitySim);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
