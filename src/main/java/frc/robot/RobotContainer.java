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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.Setpoint;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  private final ElevatorArmSubsystem elevatorArm = new ElevatorArmSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_PORT);
  final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.CODRIVER_PORT);

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    // NamedCommands.registerCommand("intakeCoral", intakeAndPivot());
    // NamedCommands.registerCommand("outtakeCoral", outtakeCoral());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("Maven Name", BuildConstants.MAVEN_NAME);
    SmartDashboard.putString("Version", BuildConstants.VERSION);
    SmartDashboard.putString("Git Sha", BuildConstants.GIT_SHA);
    SmartDashboard.putString("Git Date", BuildConstants.GIT_DATE);
    SmartDashboard.putString("Branch", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("Build Date", BuildConstants.BUILD_DATE);
    SmartDashboard.putNumber("Git Revision", BuildConstants.GIT_REVISION);
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
    driverXbox.a().onTrue(elevatorArm.setSetpoint(Setpoint.kL2));
    driverXbox.x().onTrue(elevatorArm.setSetpoint(Setpoint.kIdleSetpoint));
    driverXbox.b().whileTrue(intake.intakeGamepiece());
    driverXbox.y().onTrue(Commands.none());
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(Commands.none());
    driverXbox.leftTrigger().onTrue(Commands.none());
    driverXbox.rightTrigger().onTrue(endEffector.outtake());
    driverXbox
        .leftBumper()
        .whileTrue(
            Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.LEFT)));
    driverXbox
        .rightBumper()
        .whileTrue(
            Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.RIGHT)));

    codriverXbox.a().onTrue(Commands.none());
    codriverXbox.x().onTrue(Commands.none());
    codriverXbox.b().onTrue(Commands.none());
    codriverXbox.y().onTrue(Commands.none());
    codriverXbox.start().onTrue(Commands.none());
    codriverXbox.back().onTrue(Commands.none());
    codriverXbox.leftBumper().onTrue(Commands.none());
    codriverXbox.rightBumper().onTrue(Commands.none());
    codriverXbox.leftTrigger().onTrue(Commands.none());
    codriverXbox.rightTrigger().onTrue(Commands.none());

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
