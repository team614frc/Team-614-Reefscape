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
import frc.robot.Constants.IntakeConstants;
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
  private final IntakePivotSubsystem intakePivotLeft =
      new IntakePivotSubsystem(
          IntakeConstants.INTAKE_PIVOT_MOTOR,
          IntakeConstants.PIVOT_kP,
          IntakeConstants.PIVOT_kI,
          IntakeConstants.PIVOT_kD,
          IntakeConstants.PIVOT_kG);
  private final IntakePivotSubsystem intakePivotRight =
      new IntakePivotSubsystem(
          IntakeConstants.INTAKE_PIVOT_MOTOR,
          IntakeConstants.PIVOT_kP,
          IntakeConstants.PIVOT_kI,
          IntakeConstants.PIVOT_kD,
          IntakeConstants.PIVOT_kG);
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
        .withTimeout(duration);
  }

  private final Command toggleDriveMode =
      drivebase
          .flipFieldAndRobotRelative()
          .andThen(
              Commands.either(
                  drivebase.driveFieldOriented(driveRobotOriented),
                  drivebase.driveFieldOriented(driveAngularVelocity),
                  () -> drivebase.isFieldCentric));

  private final Command outtakeL2andL3 =
      Commands.either(
          Commands.sequence(
              arm.setSetpoint(ArmSetpoint.SCORE_L3),
              endEffector.outtake(),
              arm.setSetpoint(ArmSetpoint.PREP_L3),
              elevator.setSetpoint(ElevatorSetpoint.OUTTAKE),
              elevator.setSetpoint(ElevatorSetpoint.IDLE),
              arm.setSetpoint(ArmSetpoint.IDLE),
              endEffector.stop()),
          Commands.sequence(
              arm.setSetpoint(ArmSetpoint.SCORE_L2),
              endEffector.outtake(),
              arm.setSetpoint(ArmSetpoint.PREP_L2),
              elevator.setSetpoint(ElevatorSetpoint.IDLE),
              arm.setSetpoint(ArmSetpoint.IDLE),
              endEffector.stop()),
          (() ->
              arm.atSetpoint(ArmSetpoint.PREP_L3)
                  && elevator.atSetpoint(ElevatorSetpoint.PREP_L3)));

  private final Command autoCanalIntake =
      Commands.sequence(
          arm.setSetpoint(ArmSetpoint.PUSH),
          canal.intake(),
          elevator.setSetpoint(ElevatorSetpoint.HOVER),
          arm.setSetpoint(ArmSetpoint.HOVER),
          Commands.waitUntil(canal::gamePieceDetected),
          canal.slow(),
          Commands.waitSeconds(0.15),
          elevator.setSetpoint(ElevatorSetpoint.INTAKE).alongWith(endEffector.intake()),
          elevator.setSetpoint(ElevatorSetpoint.HOVER),
          canal.stop(),
          arm.setSetpoint(ArmSetpoint.HOVER));

  private final Command autoL1 = intake.outtakeGamepiece().withTimeout(0.25);
  private final Command autoL2 =
      arm.setSetpoint(ArmSetpoint.PUSH)
          .onlyIf(() -> arm.isBelowHorizontal())
          .andThen(
              elevator.setSetpoint(ElevatorSetpoint.PREP_L2), arm.setSetpoint(ArmSetpoint.PREP_L2));

  private final Command autoL3 =
      arm.setSetpoint(ArmSetpoint.PUSH)
          .onlyIf(() -> arm.isBelowHorizontal())
          .andThen(
              elevator.setSetpoint(ElevatorSetpoint.PREP_L3), arm.setSetpoint(ArmSetpoint.PREP_L3));

  private final Command autoElevatorArmIdle =
      arm.setSetpoint(ArmSetpoint.PUSH)
          .onlyIf(() -> arm.isBelowHorizontal())
          .andThen(elevator.setSetpoint(ElevatorSetpoint.IDLE), arm.setSetpoint(ArmSetpoint.IDLE));

  private final Command setIntakePivot(IntakePivotSetpoint setpoint) {
    return intakePivotLeft.setSetpoint(setpoint).alongWith(intakePivotRight.setSetpoint(setpoint));
  }

  private final Command autoIntakeDownAndIntake =
      setIntakePivot(IntakePivotSetpoint.DOWN).alongWith(intake.intakeGamepiece());

  private final Command autoIntakeUp = setIntakePivot(IntakePivotSetpoint.OUTTAKE_ALGAE);

  private final Command reverseClimb =
      setIntakePivot(IntakePivotSetpoint.DOWN).alongWith(climber.reverseClimb());

  private final Command climb = climber.climb().alongWith(setIntakePivot(IntakePivotSetpoint.DOWN));

  private final Command groundIntakeCoral =
      setIntakePivot(IntakePivotSetpoint.DOWN).alongWith(intake.intakeGamepiece());

  private final Command pivotIdle = setIntakePivot(IntakePivotSetpoint.OUTTAKE_ALGAE);

  private final Command outtakeGroundCoral =
      setIntakePivot(IntakePivotSetpoint.OUTTAKE_ALGAE).alongWith(intake.outtakeGamepiece());

  private final Command intakeAlgae =
      setIntakePivot(IntakePivotSetpoint.INTAKE_ALGAE).alongWith(intake.outtakeGamepiece());

  private final Command outtakeAlgae =
      setIntakePivot(IntakePivotSetpoint.OUTTAKE_ALGAE).alongWith(intake.intakeGamepiece());

  private final Command pukeOnTrue =
      Commands.parallel(
          Commands.either(
              arm.setSetpoint(ArmSetpoint.PUKE),
              arm.setSetpoint(ArmSetpoint.PUSH),
              () -> arm.atSetpoint(ArmSetpoint.HOVER)),
          canal.fast(),
          endEffector.outtake());

  private final Command pukeOnFalse =
      Commands.parallel(
          Commands.either(
              arm.setSetpoint(ArmSetpoint.HOVER),
              arm.setSetpoint(ArmSetpoint.IDLE)
                  .andThen(elevator.setSetpoint(ElevatorSetpoint.IDLE)),
              () -> arm.atSetpoint(ArmSetpoint.PUKE)),
          canal.stop(),
          endEffector.stop());

  private final Command punchL2Algae =
      Commands.sequence(
          elevator.setSetpoint(ElevatorSetpoint.PUNCH_ALGAE),
          endEffector.punchAlgae(),
          arm.setSetpoint(ArmSetpoint.PUNCH_ALGAE_L2));

  private final Command punchL3Algae =
      Commands.sequence(
          elevator.setSetpoint(ElevatorSetpoint.PUNCH_ALGAE),
          endEffector.punchAlgae(),
          arm.setSetpoint(ArmSetpoint.PUNCH_ALGAE_L3));

  private final Command canalIntakeSequence =
      Commands.sequence(
          elevator.setSetpoint(ElevatorSetpoint.HOVER),
          arm.setSetpoint(ArmSetpoint.HOVER),
          canal.intake(),
          Commands.waitUntil(canal::gamePieceDetected),
          Commands.sequence(
                  canal.slow(),
                  elevator.setSetpoint(ElevatorSetpoint.INTAKE_UP),
                  arm.setSetpoint(ArmSetpoint.INTAKE_UP),
                  endEffector.intake(),
                  elevator.setSetpoint(ElevatorSetpoint.INTAKE),
                  elevator.setSetpoint(ElevatorSetpoint.HOVER),
                  endEffector.stop(),
                  canal.stop(),
                  arm.setSetpoint(ArmSetpoint.IDLE),
                  elevator.setSetpoint(ElevatorSetpoint.IDLE))
              .alongWith(
                  rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION)));

  private final Command resetElevator =
      Commands.sequence(
          elevator.descendSlowly().until(elevator::isStalled),
          elevator.resetEncoder(),
          rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION));

  private final Command setElevatorArmIdle =
      Commands.sequence(
          arm.setSetpoint(ArmSetpoint.IDLE),
          endEffector.stop(),
          elevator.setSetpoint(ElevatorSetpoint.IDLE));

  private final Command outtakeFastGroundCoral =
      setIntakePivot(IntakePivotSetpoint.OUTTAKE_ALGAE).alongWith(intake.fastOuttakeGamepiece());

  private final Command prepL2 =
      arm.setSetpoint(ArmSetpoint.PUSH)
          .onlyIf(() -> arm.isBelowHorizontal())
          .andThen(
              elevator.setSetpoint(ElevatorSetpoint.PREP_L2), arm.setSetpoint(ArmSetpoint.PREP_L2));

  private final Command prepL3 =
      arm.setSetpoint(ArmSetpoint.PUSH)
          .onlyIf(() -> arm.isBelowHorizontal())
          .andThen(
              elevator.setSetpoint(ElevatorSetpoint.PREP_L3), arm.setSetpoint(ArmSetpoint.PREP_L3));

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
    NamedCommands.registerCommand("Score Coral", outtakeL2andL3);
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
