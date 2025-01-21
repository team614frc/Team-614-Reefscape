// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SimulationRobotConstants;

public class ElevatorSubsystem extends SubsystemBase {
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  private SparkMax leftElevatorMotor =
      new SparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
  private SparkMax rightElevatorMotor =
      new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController leftElevatorClosedLoopController =
      leftElevatorMotor.getClosedLoopController();
  private SparkClosedLoopController rightElevatorClosedLoopController =
      rightElevatorMotor.getClosedLoopController();
  // Spark Max
  private SparkMax leftArmMotor = new SparkMax(ArmConstants.LEFT_ARM_MOTOR, MotorType.kBrushless);
  private SparkMax rightArmMotor = new SparkMax(ArmConstants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
  private RelativeEncoder leftElevatorEncoder = leftElevatorMotor.getEncoder();
  private RelativeEncoder rightElevatorEncoder = rightElevatorMotor.getEncoder();
  private SparkClosedLoopController leftArmController = leftArmMotor.getClosedLoopController();
  private SparkClosedLoopController rightArmController = rightArmMotor.getClosedLoopController();

  private RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();
  private RelativeEncoder rightArmEncoder = rightArmMotor.getEncoder();

  private double elevatorCurrentTarget = ElevatorConstants.ELEVATOR_LEVEL1_SETPOINT;
  private double armCurrentTarget = ArmConstants.ARM_FEEDER_SETPOINT;

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      leftElevatorEncoder.setPosition(0);
      rightElevatorEncoder.setPosition(0);
      leftArmEncoder.setPosition(0);
      rightArmEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  // Simulation setup and variables
  private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(2);

  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);

  private DCMotor armMotorModel = DCMotor.getNEO(2);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
          SimulationRobotConstants.kArmLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));
  private final MechanismLigament2d m_armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor.configure(
        Configs.ElevatorSubsystem.leftElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightElevatorMotor.configure(
        Configs.ElevatorSubsystem.leftElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    leftArmMotor.configure(
        Configs.ElevatorSubsystem.leftArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightArmMotor.configure(
        Configs.ElevatorSubsystem.rightArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Display mechanism2d
    SmartDashboard.putData("Elevator Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    leftElevatorEncoder.setPosition(0);
    rightElevatorEncoder.setPosition(0);
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);

    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(leftElevatorMotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(leftElevatorMotor, false);
    armMotorSim = new SparkMaxSim(leftArmMotor, armMotorModel);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    leftArmController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    rightArmController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    leftElevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    rightElevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && leftElevatorMotor.getReverseLimitSwitch().isPressed()
        || rightElevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      leftArmEncoder.setPosition(0);
      rightArmEncoder.setPosition(0);
      leftElevatorEncoder.setPosition(0);
      rightElevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!leftElevatorMotor.getReverseLimitSwitch().isPressed()
        || !rightElevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              armCurrentTarget = ArmConstants.ARM_FEEDER_SETPOINT;
              elevatorCurrentTarget = ElevatorConstants.ELEVATOR_FEEDER_SETPOINT;
              break;
            case kLevel1:
              armCurrentTarget = ArmConstants.ARM_LEVEL1_SETPOINT;
              elevatorCurrentTarget = ElevatorConstants.ELEVATOR_LEVEL1_SETPOINT;
              break;
            case kLevel2:
              armCurrentTarget = ArmConstants.ARM_LEVEL2_SETPOINT;
              elevatorCurrentTarget = ElevatorConstants.ELEVATOR_LEVEL2_SETPOINT;
              break;
            case kLevel3:
              armCurrentTarget = ArmConstants.ARM_LEVEL3_SETPOINT;
              elevatorCurrentTarget = ElevatorConstants.ELEVATOR_LEVEL3_SETPOINT;
              break;
            case kLevel4:
              armCurrentTarget = ArmConstants.ARM_LEVEL4_SETPOINT;
              elevatorCurrentTarget = ElevatorConstants.ELEVATOR_LEVEL4_SETPOINT;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    zeroOnUserButton();
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    // Display subsystem values
    SmartDashboard.putNumber("Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Arm/Left Arm Actual Position", leftArmEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Right Arm Actual Position", rightArmEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber(
        "Elevator/Left Elevator Actual Position", leftElevatorEncoder.getPosition());
    SmartDashboard.putNumber(
        "Elevator/Right Elevator Actual Position", rightElevatorEncoder.getPosition());
    // Update mechanism2d
    m_elevatorMech2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * (leftElevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    leftArmEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(
        leftElevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(leftArmMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);
    // SimBattery is updated in Robot.java
  }
}
