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
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.Setpoint;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TargetingSystem;
import frc.robot.subsystems.TargetingSystem.ReefBranchSide;
import java.io.File;
import java.util.Set;
import java.util.function.DoubleSupplier;
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
  //   private final ClimberSubsystem climber = new ClimberSubsystem();
  private final CanalSubsystem canal = new CanalSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();
  private final TargetingSystem targetingSystem = new TargetingSystem();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.CODRIVER_PORT);
  private final CommandXboxController outreachXbox = new CommandXboxController(2);

  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private double getAxisOrFallback(DoubleSupplier primary, DoubleSupplier fallback) {
    return (Math.abs(primary.getAsDouble()) > 0.05)
        ? primary.getAsDouble()
        : fallback.getAsDouble();
  }

  private final SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -getAxisOrFallback(driverXbox::getLeftY, outreachXbox::getLeftY),
              () -> -getAxisOrFallback(driverXbox::getLeftX, outreachXbox::getLeftX))
          .withControllerRotationAxis(
              () -> -getAxisOrFallback(driverXbox::getRightX, outreachXbox::getRightX))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);

  private final Command driveFieldOrientedAnglularVelocity =
      drivebase.driveFieldOriented(driveAngularVelocity);

  private final SwerveInputStream driveAngularVelocitySim =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
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
      drivebase
          .flipFieldAndRobotRelative()
          .andThen(
              Commands.either(
                  drivebase.driveFieldOriented(driveRobotOriented),
                  drivebase.driveFieldOriented(driveAngularVelocity),
                  drivebase::isFieldCentric));

  private final Command outtakeCoral =
      Commands.either(
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kArmL3),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              endEffector.outtake(),
              Commands.waitSeconds(0.2),
              elevatorArm.setSetpoint(Setpoint.kArmIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              rumble(0, 0),
              endEffector.stop()),
          Commands.sequence(
              //   elevatorArm.setSetpoint(Setpoint.kScoreL2Arm),
              //   Commands.waitUntil(elevatorArm::reachedSetpoint),
              endEffector.outtake(),
              Commands.waitSeconds(0.2),
              elevatorArm.setSetpoint(Setpoint.kArmIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              rumble(0, 0),
              endEffector.stop()),
          () -> elevatorArm.checkL3());

  private final Command prepCanal =
      Commands.sequence(
          elevatorArm.setSetpoint(Setpoint.kPushArm),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          elevatorArm.setSetpoint(Setpoint.kElevatorHover),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          elevatorArm.setSetpoint(Setpoint.kArmIntake),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          elevatorArm.setSetpoint(Setpoint.kElevatorIntake));

  private final Command canalIntake =
      Commands.either(
          Commands.sequence(
              canal.intake(),
              endEffector.intake(),
              elevatorArm.setSetpoint(Setpoint.kArmIntake),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorIntake),
              canal.intake(),
              Commands.waitUntil(canal::gamePieceDetected),
              Commands.waitUntil(canal::gamePieceGone),
              Commands.waitUntil(() -> canal.gamePieceGone() && endEffector.hasGamePiece()),
              Commands.parallel(
                  rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                  canal.stop(),
                  elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                  endEffector.stop())),
          Commands.sequence(
              endEffector.intake(),
              elevatorArm.setSetpoint(Setpoint.kPushArm),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorHover),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kArmIntake),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              elevatorArm.setSetpoint(Setpoint.kElevatorIntake),
              canal.intake(),
              Commands.waitUntil(canal::gamePieceDetected),
              Commands.waitUntil(canal::gamePieceGone),
              Commands.waitUntil(() -> canal.gamePieceGone() && endEffector.hasGamePiece()),
              Commands.parallel(
                  rumble(OperatorConstants.RUMBLE_SPEED, OperatorConstants.RUMBLE_DURATION),
                  canal.stop(),
                  elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                  endEffector.stop())),
          () -> elevatorArm.isBelowHorizontal());

  private final Command autoL1 =
      Commands.sequence(
          intake.autoOuttakeGamepiece(), Commands.waitSeconds(0.2), intake.stopIntake());

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
          () -> !elevatorArm.isBelowHorizontal());

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
          () -> !elevatorArm.isBelowHorizontal());

  private final Command elevatorArmIdle =
      Commands.either(
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kArmIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              endEffector.stop(),
              elevatorArm.setSetpoint(Setpoint.kElevatorIdle)),
          Commands.sequence(
              elevatorArm.setSetpoint(Setpoint.kElevatorHover),
              elevatorArm.setSetpoint(Setpoint.kArmIdle),
              Commands.waitUntil(elevatorArm::reachedSetpoint),
              endEffector.stop(),
              elevatorArm.setSetpoint(Setpoint.kElevatorIdle)),
          () -> !elevatorArm.isBelowHorizontal());

  private final Command autoIntakeDownAndIntake =
      Commands.parallel(intakePivot.pivotDown(), intake.intakeGamepiece());

  private final Command autoIntakeUp =
      Commands.sequence(
          intakePivot.pivotIntakeAlgae(),
          Commands.waitUntil(intakePivot::reachedSetpoint),
          intakePivot.pivotIdle());

  private boolean isSlowMode = false;
  private static double SLOW_SPEED = 1.0; // m/s
  private static double NORMAL_SPEED = 17.5;

  public Command toggleSpeedModeCommand() {
    return Commands.runOnce(
        () -> {
          System.out.println("runOnce executed");

          // Toggle speed mode
          isSlowMode = !isSlowMode;
          double speed = isSlowMode ? SLOW_SPEED : NORMAL_SPEED;
          drivebase.getSwerveDrive().setMaximumAllowableSpeeds(speed, speed);

          System.out.println(
              "[Drive] Speed set to " + speed + " m/s (" + (isSlowMode ? "SLOW" : "FULL") + ")");
        });
  }

  private final Command driveReefLeft =
      Commands.sequence(
          targetingSystem.autoTargetCommand(drivebase::getPose),
          targetingSystem.setBranchSide(ReefBranchSide.LEFT),
          Commands.runOnce(
                  () -> {
                    drivebase
                        .getSwerveDrive()
                        .field
                        .getObject("target")
                        .setPose(targetingSystem.getTargetShiftPose(() -> drivebase.getPose()));
                  })
              .onlyIf(() -> targetingSystem.nearTarget(() -> drivebase.getPose())),
          Commands.defer(
                  () ->
                      drivebase.driveToPose(
                          targetingSystem.getTargetShiftPose(() -> drivebase.getPose())),
                  Set.of(drivebase))
              .onlyIf(() -> targetingSystem.nearTarget(() -> drivebase.getPose())),
          Commands.runOnce(
              () -> {
                drivebase
                    .getSwerveDrive()
                    .field
                    .getObject("target")
                    .setPose(targetingSystem.getCoralTargetPose());
              }),
          Commands.defer(
              () -> drivebase.driveToPose(targetingSystem.getCoralTargetPose()),
              Set.of(drivebase)));

  private final Command driveReefRight =
      Commands.sequence(
          targetingSystem.autoTargetCommand(drivebase::getPose),
          targetingSystem.setBranchSide(ReefBranchSide.RIGHT),
          Commands.runOnce(
                  () -> {
                    drivebase
                        .getSwerveDrive()
                        .field
                        .getObject("target")
                        .setPose(targetingSystem.getTargetShiftPose(() -> drivebase.getPose()));
                  })
              .onlyIf(() -> targetingSystem.nearTarget(() -> drivebase.getPose())),
          Commands.defer(
                  () ->
                      drivebase.driveToPose(
                          targetingSystem.getTargetShiftPose(() -> drivebase.getPose())),
                  Set.of(drivebase))
              .onlyIf(() -> targetingSystem.nearTarget(() -> drivebase.getPose())),
          Commands.runOnce(
              () -> {
                drivebase
                    .getSwerveDrive()
                    .field
                    .getObject("target")
                    .setPose(targetingSystem.getCoralTargetPose());
              }),
          Commands.defer(
              () -> drivebase.driveToPose(targetingSystem.getCoralTargetPose()),
              Set.of(drivebase)));

  private final Command punchL3Algae =
      Commands.sequence(
          elevatorArm.setSetpoint(Setpoint.kOuttakeElevatorAlgae),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          endEffector.punchAlgae(),
          elevatorArm.setSetpoint(Setpoint.kOuttakeArmAlgaeL3));

  private final Command punchL2Algae =
      Commands.sequence(
          elevatorArm.setSetpoint(Setpoint.kElevatorL2),
          Commands.waitUntil(elevatorArm::reachedSetpoint),
          endEffector.punchAlgae(),
          elevatorArm.setSetpoint(Setpoint.kOuttakeArmAlgaeL3));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // canal.setDefaultCommand(canal.intake());

    NamedCommands.registerCommand("L1", autoL1);
    NamedCommands.registerCommand("Stop Ground Intake", intake.stopIntake());
    NamedCommands.registerCommand("L2", autoL2);
    NamedCommands.registerCommand("L3", autoL3);
    NamedCommands.registerCommand("driveReefLeft", driveReefLeft);
    NamedCommands.registerCommand("driveReefRight", driveReefRight);
    NamedCommands.registerCommand("L3", autoL3);
    NamedCommands.registerCommand("Canal Intake", canalIntake);
    NamedCommands.registerCommand("Score Coral", outtakeCoral);
    NamedCommands.registerCommand("Idle Setpoint", elevatorArmIdle);
    NamedCommands.registerCommand("Intake Down", autoIntakeDownAndIntake);
    NamedCommands.registerCommand("Intake Up", autoIntakeUp);
    NamedCommands.registerCommand("Prep Canal", prepCanal);
    NamedCommands.registerCommand("Punch L3 Algae", punchL3Algae);
    NamedCommands.registerCommand("Punch L2 Algae", punchL2Algae);

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("Forward");
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
    driverXbox.x().whileTrue(driveReefLeft);
    driverXbox.b().whileTrue(driveReefRight);
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.back().onTrue(toggleDriveMode);
    driverXbox.leftBumper().whileTrue(intake.passthrough());
    // driverXbox.a().whileTrue(Commands.parallel(intakePivot.pivotDown(), climber.reverseClimb()));
    // driverXbox.y().whileTrue(Commands.parallel(climber.climb(), intakePivot.pivotDown()));
    driverXbox
        .leftTrigger()
        .whileTrue(Commands.parallel(intakePivot.pivotDown(), intake.intakeGamepiece()))
        .onFalse(Commands.sequence(intakePivot.pivotOuttakeAlgae()));
    driverXbox
        .rightTrigger()
        .whileTrue(Commands.parallel(intakePivot.pivotOuttakeAlgae(), intake.outtakeGamepiece()));
    driverXbox
        .a()
        .whileTrue(Commands.parallel(intakePivot.pivotIntakeAlgae(), intake.intakeAlgae()))
        .onFalse(intakePivot.pivotOuttakeAlgae());
    driverXbox
        .y()
        .whileTrue(Commands.parallel(intakePivot.pivotOuttakeAlgae(), intake.outtakeAlgae()))
        .onFalse(intakePivot.pivotOuttakeAlgae());
    driverXbox.rightBumper().onTrue(outtakeCoral);
    driverXbox
        .povUp()
        .whileTrue(
            Commands.either(
                Commands.sequence(
                    elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                    elevatorArm.setSetpoint(Setpoint.kPuke),
                    canal.fast(),
                    endEffector.outtake()),
                Commands.parallel(
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    canal.fast(),
                    endEffector.outtake()),
                () -> elevatorArm.isBelowHorizontal()))
        .onFalse(
            Commands.either(
                Commands.sequence(
                    canal.stop(),
                    endEffector.stop(),
                    elevatorArm.setSetpoint(Setpoint.kArmIntake),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorIntake)),
                Commands.sequence(
                    canal.stop(),
                    endEffector.stop(),
                    elevatorArm.setSetpoint(Setpoint.kArmIdle),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorIdle)),
                () -> elevatorArm.checkPuke()));

    codriverXbox.a().onTrue(punchL2Algae);
    codriverXbox
        .y()
        .onTrue(
            Commands.sequence(
                elevatorArm.setSetpoint(Setpoint.kElevatorL3),
                Commands.waitUntil(elevatorArm::reachedSetpoint),
                endEffector.punchAlgae(),
                elevatorArm.setSetpoint(Setpoint.kOuttakeArmAlgaeL3)));
    codriverXbox.x().onTrue(canalIntake);
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
    codriverXbox.back().onTrue(Commands.none());
    codriverXbox.leftBumper().onTrue(elevatorArmIdle);
    codriverXbox
        .rightBumper()
        .whileTrue(
            Commands.parallel(intakePivot.pivotOuttakeAlgae(), intake.fastOuttakeGamepiece()));
    codriverXbox
        .leftTrigger()
        .onTrue(
            Commands.either(
                Commands.sequence(
                    endEffector.stop(),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL2),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL2)),
                Commands.sequence(
                    endEffector.stop(),
                    elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL2),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL2)),
                () -> !elevatorArm.isBelowHorizontal()));
    codriverXbox
        .rightTrigger()
        .onTrue(
            Commands.either(
                Commands.sequence(
                    endEffector.stop(),
                    elevatorArm.setSetpoint(Setpoint.kElevatorHover),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL3),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL3)),
                Commands.sequence(
                    endEffector.stop(),
                    elevatorArm.setSetpoint(Setpoint.kPushArm),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kElevatorL3),
                    Commands.waitUntil(elevatorArm::reachedSetpoint),
                    elevatorArm.setSetpoint(Setpoint.kArmL3)),
                () -> elevatorArm.isBelowHorizontal()));

    outreachXbox.b().onTrue(toggleSpeedModeCommand());

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
