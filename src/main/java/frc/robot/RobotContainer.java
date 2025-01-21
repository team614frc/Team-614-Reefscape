// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;

// import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController codriverXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  //  private final SwerveSubsystem drivebase =
  // new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  /* SwerveInputStream driveAngularVelocity =
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
      drivebase.driveFieldOriented(driveAngularVelocitySim); */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    // SmartDashboard.putData("Auto Chooser", autoChooser);
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
    driverXbox.a().onTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kLevel2));
    driverXbox.x().onTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kLevel3));
    driverXbox.y().onTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kLevel4));
    driverXbox.b().onTrue(Commands.none());
    // driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
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

    /* drivebase.setDefaultCommand(
    !RobotBase.isSimulation()
        ? driveFieldOrientedAnglularVelocity
        : driveFieldOrientedAngularVelocitySim); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {

  // }

  public void setDriveMode() {
    configureBindings();
  }

  /* public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  } */
}
