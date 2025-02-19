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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.Setpoint;
import frc.robot.subsystems.EndEffectorSubsystem;
// import frc.robot.subsystems.IntakePivotSubsystem;
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
  // private final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
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

  /**
   * This command will only run when the end effector does not have a game piece, this keeps the
   * canal running until laserCAN sees the coral
   */
  private Command pivotAndIntake() {
    Command intakeCommand =
        Commands.parallel(
            led.setIntakePattern(),
            // intakePivot.pivotDown(),
            intake.intakeGamepiece(),
            canal.intake(),
            endEffector.intake(),
            elevatorArm.setSetpoint(Setpoint.kHover));

    Command releaseCommand = Commands.parallel(intake.stopIntake());
    // intakePivot.pivotUp());

    Command canalDetectionParallel =
        Commands.waitUntil(canal::gamePieceDetected)
            .andThen(
                Commands.parallel(
                    canal.stop(),
                    rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                    elevatorArm.setSetpoint(Setpoint.kIntake)));

    Command endEffectorDetection =
        Commands.waitUntil(endEffector::hasGamePiece)
            .andThen(led.setBasicPattern(), elevatorArm.setSetpoint(Setpoint.kIdleSetpoint));

    Command startEndCommand =
        Commands.startEnd(
            () -> intakeCommand.schedule(),
            () -> {
              releaseCommand.schedule();
              canalDetectionParallel.schedule();
              endEffectorDetection.schedule();
            });

    return new ConditionalCommand(
        Commands.none(),
        startEndCommand,
        () -> endEffector.hasGamePiece() && canal.gamePieceDetected());
  }

  private final Command outtakeCoral =
      Commands.sequence(
          endEffector.outtake().withTimeout(0.2),
          endEffector.stop(),
          elevatorArm.setSetpoint(Setpoint.kIdleSetpoint),
          elevatorArm.setSetpoint(Setpoint.kHover).withTimeout(1.5));

  private final Command autoIntake() {
    Command startIntake =
        Commands.parallel(
            led.setIntakePattern(),
            // intakePivot.pivotDown(),
            intake.intakeGamepiece(),
            canal.intake(),
            endEffector.intake(),
            elevatorArm.setSetpoint(Setpoint.kHover));

    Command stopIntake =
        Commands.parallel(
            intake.stopIntake(),
            // intakePivot.pivotUp(),
            canal.stop(),
            elevatorArm.setSetpoint(Setpoint.kIntake));

    Command endEffectorDetection =
        Commands.waitUntil(endEffector::hasGamePiece)
            .andThen(led.setBasicPattern(), elevatorArm.setSetpoint(Setpoint.kHover));

    return Commands.sequence(
        startIntake,
        Commands.waitUntil(canal::gamePieceDetected).andThen(stopIntake),
        endEffectorDetection);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    NamedCommands.registerCommand("L1", elevatorArm.setSetpoint(Setpoint.kL1));
    NamedCommands.registerCommand("L2", elevatorArm.setSetpoint(Setpoint.kL2));
    NamedCommands.registerCommand("L3", elevatorArm.setSetpoint(Setpoint.kL3));
    NamedCommands.registerCommand("Intake", autoIntake());
    NamedCommands.registerCommand("Outtake", outtakeCoral);

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
    driverXbox.a().onTrue(elevatorArm.setSetpoint(Setpoint.kL3));
    driverXbox
        .x()
        .whileTrue(
            Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.LEFT)));
    driverXbox
        .b()
        .whileTrue(
            Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.RIGHT)));
    driverXbox.y().onTrue(elevatorArm.setSetpoint(Setpoint.kIdleSetpoint));
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(Commands.none());
    driverXbox.leftBumper().whileTrue(climber.reverseClimb());
    driverXbox.rightBumper().whileTrue(climber.climb());
    driverXbox.leftTrigger().whileTrue(pivotAndIntake());
    driverXbox.rightTrigger().onTrue(outtakeCoral);

    codriverXbox.a().onTrue(elevatorArm.setSetpoint(Setpoint.kL1));
    codriverXbox.x().onTrue(elevatorArm.setSetpoint(Setpoint.kL2));
    codriverXbox
        .b()
        .onTrue(
            Commands.sequence(
                new ParallelCommandGroup(
                        elevatorArm.setSetpoint(Setpoint.kIntake), endEffector.intake())
                    .until(elevatorArm::atSetpoint),
                endEffector.stall(),
                elevatorArm.setSetpoint(Setpoint.kHover).until(elevatorArm::atSetpoint),
                new ParallelCommandGroup(
                    endEffector.stop(), elevatorArm.setSetpoint(Setpoint.kIdleSetpoint))));
    codriverXbox.y().onTrue(endEffector.intake());
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
