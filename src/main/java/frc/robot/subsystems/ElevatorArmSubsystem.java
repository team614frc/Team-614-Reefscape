// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

public class ElevatorArmSubsystem extends SubsystemBase {
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  private SparkFlex elevatorMotor =
      new SparkFlex(ElevatorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();

  private SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  private double elevatorCurrentTarget = ElevatorConstants.ELEVATOR_FEEDER_SETPOINT;
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
      elevatorEncoder.setPosition(0);
      armEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  // Simulation setup and variables
  private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);

  private SparkFlexSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.ELEVATOR_GEARING,
          SimulationRobotConstants.CARRIAGE_MASS,
          SimulationRobotConstants.ELEVATOR_DRUM_RADIUS,
          SimulationRobotConstants.MIN_ELEVATORHEIGHT_METERS,
          SimulationRobotConstants.MAX_ELEVATORHEIGHT_METERS,
          true,
          SimulationRobotConstants.MIN_ELEVATORHEIGHT_METERS);

  private DCMotor armMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.ARM_REDUCTION,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.ARM_LENGTH, SimulationRobotConstants.ARM_MASS),
          SimulationRobotConstants.ARM_LENGTH,
          SimulationRobotConstants.MIN_ANGLE_RADS,
          SimulationRobotConstants.MAX_ANGLE_RADS,
          true,
          SimulationRobotConstants.MIN_ANGLE_RADS);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.MIN_ELEVATORHEIGHT_METERS
                  * SimulationRobotConstants.PIXELSPERMETER,
              90));
  private final MechanismLigament2d m_armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.ARM_LENGTH * SimulationRobotConstants.PIXELSPERMETER,
              180 - Units.radiansToDegrees(SimulationRobotConstants.MIN_ANGLE_RADS) - 90));

  /** Creates a new ElevatorArmSubsystem. */
  public ElevatorArmSubsystem() {

    elevatorMotor.configure(
        Configs.ElevatorArmSubsystem.ARM_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    armMotor.configure(
        Configs.ElevatorArmSubsystem.ARM_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Display mechanism2d
    SmartDashboard.putData("Elevator Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    elevatorEncoder.setPosition(0);
    armEncoder.setPosition(0);

    // Initialize simulation values
    elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
    armMotorSim = new SparkFlexSim(armMotor, armMotorModel);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
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
    SmartDashboard.putNumber("Arm/Arm Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator/Elevator Actual Position", elevatorEncoder.getPosition());

    // Update mechanism2d
    m_elevatorMech2d.setLength(
        SimulationRobotConstants.PIXELSPERMETER * SimulationRobotConstants.MIN_ELEVATORHEIGHT_METERS
            + SimulationRobotConstants.PIXELSPERMETER
                * (elevatorEncoder.getPosition() / SimulationRobotConstants.ELEVATOR_GEARING)
                * (SimulationRobotConstants.ELEVATOR_DRUM_RADIUS * 2.0 * Math.PI));
    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.MIN_ANGLE_RADS)
                + Units.rotationsToDegrees(
                    armEncoder.getPosition() / SimulationRobotConstants.ARM_REDUCTION))
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
    m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.ELEVATOR_DRUM_RADIUS * 2.0 * Math.PI))
                * SimulationRobotConstants.ELEVATOR_GEARING)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.ARM_REDUCTION),
        RobotController.getBatteryVoltage(),
        0.02);
    // SimBattery is updated in Robot.java
  }
}
