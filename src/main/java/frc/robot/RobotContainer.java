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
import frc.robot.Constants.ArmSetpoint;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.Constants.IntakePivotSetpoint;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CanalSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final CanalSubsystem canal = new CanalSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.CODRIVER_PORT);

  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivebase =
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
    return Commands.runEnd(
            () -> {
              driverXbox.getHID().setRumble(RumbleType.kBothRumble, power);
              codriverXbox.getHID().setRumble(RumbleType.kBothRumble, power);
            },
            () -> {
              driverXbox.getHID().setRumble(RumbleType.kBothRumble, OperatorConstants.RUMBLE_REST);
              codriverXbox
                  .getHID()
                  .setRumble(RumbleType.kBothRumble, OperatorConstants.RUMBLE_REST);
            })
        .withDeadline(Commands.waitSeconds(duration));
  }

  private final Command toggleDriveMode =
      Commands.sequence(
              drivebase.driveFieldOriented(driveRobotOriented),
              drivebase.flipFieldAndRobotRelative())
          .onlyIf(() -> drivebase.isFieldCentric)
          .andThen(
              Commands.sequence(
                  drivebase.driveFieldOriented(driveAngularVelocitySim),
                  drivebase.flipFieldAndRobotRelative()));

  private final Command autoOuttakeCoral =
      Commands.sequence(
              arm.setSetpoint(ArmSetpoint.PREP_L3).until(() -> arm.atSetpoint()),
              endEffector.outtake(),
              arm.setSetpoint(ArmSetpoint.PREP_L3),
              elevator.setSetpoint(ElevatorSetpoint.OUTTAKE).until(() -> elevator.atSetpoint()),
              elevator.setSetpoint(ElevatorSetpoint.IDLE),
              arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
              endEffector.stop())
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.sequence(
                  arm.setSetpoint(ArmSetpoint.PREP_L2).until(() -> arm.atSetpoint()),
                  endEffector.outtake(),
                  arm.setSetpoint(ArmSetpoint.PREP_L2),
                  elevator.setSetpoint(ElevatorSetpoint.IDLE).until(() -> elevator.atSetpoint()),
                  arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
                  endEffector.stop()));

  private final Command autoCanalIntake =
      Commands.sequence(
          arm.setSetpoint(ArmSetpoint.PUSH).until(() -> arm.atSetpoint()),
          canal.intake(),
          elevator.setSetpoint(ElevatorSetpoint.HOVER).until(() -> elevator.atSetpoint()),
          arm.setSetpoint(ArmSetpoint.HOVER).until(canal::gamePieceDetected),
          canal.slow(),
          Commands.waitSeconds(0.15),
          Commands.parallel(elevator.setSetpoint(ElevatorSetpoint.INTAKE), endEffector.intake())
              .until(() -> elevator.atSetpoint()),
          elevator.setSetpoint(ElevatorSetpoint.HOVER).until(() -> elevator.atSetpoint()),
          canal.stop(),
          arm.setSetpoint(ArmSetpoint.HOVER));

  private final Command autoL1 =
      Commands.sequence(
          intake.autoOuttakeGamepiece(), Commands.waitSeconds(0.25), intake.stopIntake());

  private final Command autoL2 =
      Commands.sequence(arm.setSetpoint(ArmSetpoint.PUSH).until(() -> arm.atSetpoint()))
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              elevator.setSetpoint(ElevatorSetpoint.PREP_L2).until(() -> elevator.atSetpoint()),
              arm.setSetpoint(ArmSetpoint.PREP_L2));

  private final Command autoL3 =
      Commands.sequence(arm.setSetpoint(ArmSetpoint.PUSH).until(() -> arm.atSetpoint()))
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.sequence(
                  elevator.setSetpoint(ElevatorSetpoint.PREP_L3).until(() -> elevator.atSetpoint()),
                  arm.setSetpoint(ArmSetpoint.PREP_L3)));

  private final Command autoElevatorArmIdle =
      Commands.sequence(
          led.setBasicPattern(),
          arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
          elevator.setSetpoint(ElevatorSetpoint.IDLE));

  private final Command autoIntakeDownAndIntake =
      Commands.parallel(
          intakePivot.setSetpoint(IntakePivotSetpoint.DOWN), intake.intakeGamepiece());

  private final Command autoIntakeUp =
      Commands.sequence(intakePivot.setSetpoint(IntakePivotSetpoint.OUTTAKE_ALGAE));

  private final Command reverseClimb =
      Commands.parallel(intakePivot.setSetpoint(IntakePivotSetpoint.DOWN), climber.reverseClimb());

  private final Command climb =
      Commands.parallel(climber.climb(), intakePivot.setSetpoint(IntakePivotSetpoint.DOWN));

  private final Command groundIntakeCoral =
      Commands.parallel(
          intakePivot.setSetpoint(IntakePivotSetpoint.DOWN), intake.intakeGamepiece());

  private final Command pivotIdle = intakePivot.setSetpoint(IntakePivotSetpoint.OUTTAKE_ALGAE);

  private final Command outtakeGroundCoral =
      Commands.parallel(
          intakePivot.setSetpoint(IntakePivotSetpoint.OUTTAKE_ALGAE), intake.outtakeGamepiece());

  private final Command intakeAlgae =
      Commands.parallel(
          intakePivot.setSetpoint(IntakePivotSetpoint.INTAKE_ALGAE), intake.outtakeGamepiece());

  private final Command outtakeAlgae =
      Commands.parallel(
          intakePivot.setSetpoint(IntakePivotSetpoint.OUTTAKE_ALGAE), intake.intakeGamepiece());

  private final Command outtakeL2andL3 =
      Commands.sequence(
              arm.setSetpoint(ArmSetpoint.PREP_L3).until(() -> arm.atSetpoint()),
              endEffector.outtake(),
              arm.setSetpoint(ArmSetpoint.PREP_L3),
              elevator.setSetpoint(ElevatorSetpoint.OUTTAKE).until(() -> elevator.atSetpoint()),
              elevator.setSetpoint(ElevatorSetpoint.IDLE),
              arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
              endEffector.stop())
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.sequence(
                  arm.setSetpoint(ArmSetpoint.PREP_L2).until(() -> arm.atSetpoint()),
                  endEffector.outtake(),
                  arm.setSetpoint(ArmSetpoint.PREP_L2),
                  elevator.setSetpoint(ElevatorSetpoint.IDLE).until(() -> elevator.atSetpoint()),
                  arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
                  endEffector.stop()));

  private final Command pukeOnTrue =
      Commands.parallel(arm.setSetpoint(ArmSetpoint.PUKE), canal.fast(), endEffector.outtake())
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.parallel(
                  arm.setSetpoint(ArmSetpoint.PUSH), canal.fast(), endEffector.outtake()));

  private final Command pukeOnFalse =
      Commands.parallel(arm.setSetpoint(ArmSetpoint.HOVER), canal.stop(), endEffector.stop())
          .onlyIf(() -> arm.atSetpoint())
          .andThen(
              Commands.sequence(
                  canal.stop(),
                  endEffector.stop(),
                  arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
                  elevator.setSetpoint(ElevatorSetpoint.IDLE)));

  private final Command punchL2Algae =
      Commands.sequence(
          elevator.setSetpoint(ElevatorSetpoint.PUNCH_ALGAE).until(() -> elevator.atSetpoint()),
          endEffector.punchAlgae(),
          arm.setSetpoint(ArmSetpoint.PUNCH_ALGAE_L2));

  private final Command punchL3Algae =
      Commands.sequence(
          elevator.setSetpoint(ElevatorSetpoint.PUNCH_ALGAE).until(() -> elevator.atSetpoint()),
          endEffector.punchAlgae(),
          arm.setSetpoint(ArmSetpoint.PUNCH_ALGAE_L3));

  private final Command canalIntakeSequence =
      Commands.sequence(
              canal.intake().until(canal::gamePieceDetected),
              Commands.parallel(
                  rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                  canal.slow()),
              Commands.waitSeconds(0.05),
              Commands.parallel(elevator.setSetpoint(ElevatorSetpoint.INTAKE), endEffector.intake())
                  .until(() -> elevator.atSetpoint()),
              elevator.setSetpoint(ElevatorSetpoint.HOVER).until(() -> elevator.atSetpoint()),
              endEffector.stop(),
              canal.stop(),
              arm.setSetpoint(ArmSetpoint.HOVER))
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.sequence(
                  arm.setSetpoint(ArmSetpoint.PUSH).until(() -> arm.atSetpoint()),
                  canal.intake(),
                  elevator.setSetpoint(ElevatorSetpoint.HOVER).until(() -> elevator.atSetpoint()),
                  arm.setSetpoint(ArmSetpoint.HOVER).until(canal::gamePieceDetected),
                  Commands.parallel(
                      rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                      canal.slow()),
                  Commands.waitSeconds(0.05),
                  Commands.parallel(
                          elevator.setSetpoint(ElevatorSetpoint.INTAKE), endEffector.intake())
                      .until(() -> elevator.atSetpoint()),
                  elevator.setSetpoint(ElevatorSetpoint.HOVER).until(() -> elevator.atSetpoint()),
                  endEffector.stop(),
                  canal.stop(),
                  arm.setSetpoint(ArmSetpoint.HOVER)));

  private final Command resetElevator =
      Commands.sequence(
          elevator.descendSlowly().until(elevator::isStalled),
          elevator.resetEncoder(),
          rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION));

  private final Command setElevatorArmIdle =
      Commands.sequence(
          arm.setSetpoint(ArmSetpoint.IDLE).until(() -> arm.atSetpoint()),
          endEffector.stop(),
          elevator.setSetpoint(ElevatorSetpoint.IDLE));

  private final Command outtakeFastGroundCoral =
      Commands.parallel(
          intakePivot.setSetpoint(IntakePivotSetpoint.OUTTAKE_ALGAE),
          intake.fastOuttakeGamepiece());

  private final Command prepL2 =
      Commands.sequence(arm.setSetpoint(ArmSetpoint.PUSH).until(() -> arm.atSetpoint()))
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.sequence(
                  elevator.setSetpoint(ElevatorSetpoint.PREP_L2).until(() -> elevator.atSetpoint()),
                  arm.setSetpoint(ArmSetpoint.PREP_L2)));

  private final Command prepL3 =
      Commands.sequence(arm.setSetpoint(ArmSetpoint.PUSH).until(() -> arm.atSetpoint()))
          .onlyIf(() -> arm.atSetpoint() && elevator.atSetpoint())
          .andThen(
              Commands.sequence(
                  elevator.setSetpoint(ElevatorSetpoint.PREP_L3).until(() -> elevator.atSetpoint()),
                  arm.setSetpoint(ArmSetpoint.PREP_L3)));

  private final Command driveLeftReef =
      Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.LEFT));

  private final Command driveRightReef =
      Commands.deferredProxy(() -> drivebase.driveReef(FieldConstants.Direction.RIGHT));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // canal.setDefaultCommand(canal.intake());

    NamedCommands.registerCommand("L1", autoL1);
    NamedCommands.registerCommand("Stop Ground Intake", intake.stopIntake());
    NamedCommands.registerCommand("L2", autoL2);
    NamedCommands.registerCommand("L3", autoL3);
    NamedCommands.registerCommand("Canal Intake", autoCanalIntake);
    NamedCommands.registerCommand("Score Coral", autoOuttakeCoral);
    NamedCommands.registerCommand("Idle Setpoint", autoElevatorArmIdle);
    NamedCommands.registerCommand("Intake Down", autoIntakeDownAndIntake);
    NamedCommands.registerCommand("Intake Up", autoIntakeUp);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Git Revision", BuildConstants.GIT_REVISION);
    SmartDashboard.putString("Git Sha", BuildConstants.GIT_SHA);
    SmartDashboard.putString("Git Date", BuildConstants.GIT_DATE);
    SmartDashboard.putString("Git Branch", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("Build Date", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("Version", BuildConstants.VERSION);
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
    // driverXbox.x().whileTrue(driveLeftReef);
    // driverXbox.b().whileTrue(driveRightReef);
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(toggleDriveMode);
    driverXbox.a().whileTrue(reverseClimb);
    driverXbox.y().whileTrue(climb);
    driverXbox.leftTrigger().whileTrue(groundIntakeCoral).onFalse(pivotIdle);
    driverXbox.rightTrigger().whileTrue(outtakeGroundCoral);
    driverXbox.x().whileTrue(intakeAlgae).onFalse(pivotIdle);
    driverXbox.b().whileTrue(outtakeAlgae).onFalse(pivotIdle);
    driverXbox.rightBumper().onTrue(outtakeL2andL3);
    driverXbox.leftBumper().whileTrue(pukeOnTrue).onFalse(pukeOnFalse);

    codriverXbox.a().onTrue(punchL2Algae);
    codriverXbox.y().onTrue(punchL3Algae);
    codriverXbox.x().onTrue(canalIntakeSequence);
    codriverXbox.start().whileTrue(resetElevator);
    codriverXbox.leftBumper().onTrue(setElevatorArmIdle);
    codriverXbox.rightBumper().whileTrue(outtakeFastGroundCoral);
    codriverXbox.leftTrigger().onTrue(prepL2);
    codriverXbox.rightTrigger().onTrue(prepL3);

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
