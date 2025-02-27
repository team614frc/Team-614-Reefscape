// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.Setpoint;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
  private final CanalSubsystem canal = new CanalSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_PORT);
  final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.CODRIVER_PORT);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> driverXbox.getLeftY(), () -> driverXbox.getLeftX())
          .withControllerRotationAxis(() -> -driverXbox.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);

  private final Command driveFieldOrientedAnglularVelocity =
      drivebase.driveFieldOriented(driveAngularVelocity);

  private final SwerveInputStream driveAngularVelocitySim =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> driverXbox.getLeftY(), () -> driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(4))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);

  private final Command driveFieldOrientedAngularVelocitySim =
      drivebase.driveFieldOriented(driveAngularVelocitySim);

  private final SwerveInputStream driveRobotOriented =
      driveAngularVelocitySim.copy().robotRelative(true).allianceRelativeControl(false);

  private final Command rumble(double power, double duration) {
    return Commands.runOnce(
            () -> {
              driverXbox.getHID().setRumble(RumbleType.kBothRumble, power);
              codriverXbox.getHID().setRumble(RumbleType.kBothRumble, power);
            })
        .andThen(Commands.waitSeconds(duration))
        .andThen(
            () -> {
              driverXbox.getHID().setRumble(RumbleType.kBothRumble, OperatorConstants.RUMBLE_REST);
              codriverXbox
                  .getHID()
                  .setRumble(RumbleType.kBothRumble, OperatorConstants.RUMBLE_REST);
            });
  }

  private final Command toggleDriveMode =
      Commands.either(
          Commands.parallel(
              drivebase.driveFieldOriented(driveRobotOriented),
              drivebase.flipFieldAndRobotRelative()),
          Commands.parallel(
              drivebase.driveFieldOriented(driveAngularVelocitySim),
              drivebase.flipFieldAndRobotRelative()),
          () -> drivebase.isFieldCentric);

  private final Command autoOuttakeCoral =
      Commands.either(
          Commands.sequence(
              endEffector.outtake(),
              elevatorArm.setSetpoint(Setpoint.kScoreL3Arm),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmL3),
              elevatorArm.setSetpoint(Setpoint.kElevatorOuttake),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorIdle),
              elevatorArm.setSetpoint(Setpoint.kArmIdle),
              endEffector.stop()),
          Commands.sequence(
              endEffector.outtake(),
              elevatorArm.setSetpoint(Setpoint.kScoreL2Arm),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmL2),
              elevatorArm.setSetpoint(Setpoint.kElevatorIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmIdle),
              endEffector.stop()),
          () -> elevatorArm.checkL3());

  private final Command autoIntake =
      Commands.sequence(
          elevatorArm.setSetpoint(Setpoint.kPushArm),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          canal.intake(),
          elevatorArm.setSetpoint(Setpoint.kElevatorHover),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          elevatorArm.setSetpoint(Setpoint.kArmHover),
          Commands.waitUntil(canal::gamePieceDetected),
          canal.slow(),
          Commands.waitSeconds(0.8),
          Commands.parallel(elevatorArm.setSetpoint(Setpoint.kIntake), endEffector.intake()),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          elevatorArm.setSetpoint(Setpoint.kElevatorHover),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          canal.stop(),
          elevatorArm.setSetpoint(Setpoint.kArmHover));

  private final Command autoL2 =
      Commands.either(
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kElevatorL2),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmL2)),
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kPushArm),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorL2),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmL2)),
          () -> elevatorArm.armSetpointComparison());

  private final Command autoL3 =
      Commands.either(
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kElevatorL3),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmL3)),
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kPushArm),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorL3),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmL3)),
          () -> elevatorArm.armSetpointComparison());

  private final Command autoIdle =
      Commands.sequence(
          led.setBasicPattern(),
          elevatorArm.setSetpoint(Setpoint.kArmIdle),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          elevatorArm.setSetpoint(Setpoint.kElevatorIdle));
  private final Command L1Outtake = Commands.parallel(intake.outtakeGamepiece());
  private final Command stopIntake = Commands.parallel(intake.stopIntake());

  private final Command driveLeftReef =
      Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.LEFT));

  private final Command driveRightReef =
      Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.RIGHT));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // canal.setDefaultCommand(canal.intake());

    NamedCommands.registerCommand("L1", L1Outtake);
    NamedCommands.registerCommand("Stop Ground Intake", stopIntake);
    NamedCommands.registerCommand("L2", autoL2);
    NamedCommands.registerCommand("L3", autoL3);
    NamedCommands.registerCommand("Canal Intake", autoIntake);
    NamedCommands.registerCommand("Score Coral", autoOuttakeCoral);
    NamedCommands.registerCommand("Idle Setpoint", autoIdle);

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
    driverXbox.x().whileTrue(driveLeftReef);
    driverXbox.b().whileTrue(driveRightReef);
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(toggleDriveMode);
    driverXbox.a().whileTrue(climber.reverseClimb());
    driverXbox.y().whileTrue(Commands.parallel(climber.climb(), intakePivot.pivotDown()));
    driverXbox
        .leftTrigger()
        .whileTrue(Commands.parallel(intakePivot.pivotDown(), intake.intakeGamepiece()))
        .onFalse(intakePivot.pivotIdle());
    driverXbox
        .leftBumper()
        .whileTrue(Commands.parallel(intakePivot.pivotAlgae(), intake.intakeGamepiece()))
        .onFalse(intakePivot.pivotIdle());
    driverXbox
        .rightTrigger()
        .whileTrue(Commands.parallel(intakePivot.pivotAlgae(), intake.outtakeGamepiece()))
        .onFalse(intakePivot.pivotIdle());
    driverXbox
        .rightBumper()
        .onTrue(
            Commands.either(
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kScoreL3Arm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    endEffector.outtake(),
                    elevatorArm.setSetpoint(Setpoint.kArmL3),
                    elevatorArm.setSetpoint(Setpoint.kElevatorOuttake),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorIdle),
                    elevatorArm.setSetpoint(Setpoint.kArmIdle),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    endEffector.stop()),
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kScoreL2Arm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    endEffector.outtake(),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL2),
                    elevatorArm.setSetpoint(Setpoint.kElevatorIdle),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmIdle),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    endEffector.stop()),
                () -> elevatorArm.checkL3()));
    codriverXbox.a().onTrue(intakePivot.pivotDown());
    codriverXbox.b().onTrue(intakePivot.pivotAlgae());
    codriverXbox.y().onTrue(intakePivot.pivotIdle());

    codriverXbox
        .x()
        .onTrue(
            Commands.either(
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    canal.intake(),
                    elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmHover),
                    Commands.waitUntil(canal::gamePieceDetected),
                    Commands.parallel(
                        rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                        canal.slow()),
                    Commands.waitSeconds(0.65),
                    Commands.parallel(
                        elevatorArm.setSetpoint(Setpoint.kIntake), endEffector.intake()),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    endEffector.stop(),
                    canal.stop(),
                    elevatorArm.setSetpoint(Setpoint.kArmHover)),
                Commands.sequence(
                    Commands.waitUntil(canal::gamePieceDetected),
                    Commands.parallel(
                        rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                        canal.slow()),
                    Commands.waitSeconds(0.65),
                    Commands.parallel(
                        elevatorArm.setSetpoint(Setpoint.kIntake), endEffector.intake()),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    endEffector.stop(),
                    canal.stop(),
                    elevatorArm.setSetpoint(Setpoint.kArmHover)),
                () -> elevatorArm.checkHover()));
    codriverXbox
        .start()
        .whileTrue(
            Commands.sequence(
                elevatorArm.setElevatorResetSpeed(),
                Commands.waitSeconds(1),
                Commands.waitUntil(elevatorArm::elevatorStalled),
                Commands.waitSeconds(0.25),
                elevatorArm.resetElevatorEncoder(),
                rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION)));
    codriverXbox
        .leftBumper()
        .onTrue(
            Commands.sequence(
                elevatorArm.setSetpoint(Setpoint.kArmIdle),
                Commands.waitUntil(elevatorArm::reachedSetpoint),
                endEffector.stop(),
                elevatorArm.setSetpoint(Setpoint.kElevatorIdle)));
    codriverXbox.rightBumper().onTrue(Commands.none());
    codriverXbox
        .leftTrigger()
        .onTrue(
            Commands.either(
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kElevatorL2),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL2)),
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL2),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL2)),
                () -> elevatorArm.armSetpointComparison()));
    codriverXbox
        .rightTrigger()
        .onTrue(
            Commands.either(
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL3),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL3)),
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL3),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL3)),
                () -> elevatorArm.armSetpointComparison()));

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
