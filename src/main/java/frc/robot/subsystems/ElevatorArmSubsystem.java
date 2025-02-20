// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    kHover,
    kIntake,
    kIdleSetpoint,
    kL1,
    kL2,
    kL3,
    kL4;
  }

  // Elevator Motor
  private SparkFlex elevatorMotor =
      new SparkFlex(ElevatorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getExternalEncoder();

  // Arm Motor
  private SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
  private SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder();

  private ProfiledPIDController pid =
      new ProfiledPIDController(
          ArmConstants.kP,
          ArmConstants.kI,
          ArmConstants.kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(ArmConstants.ARM_MAX_VELOCITY),
              Units.rotationsToRadians(ArmConstants.ARM_MAX_ACCELERATION)));

  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  // Default Current Target
  private double elevatorSetpoint = ElevatorConstants.ELEVATOR_IDLE_SETPOINT;
  private double armSetpoint = ArmConstants.ARM_IDLE_SETPOINT;

  // Simulation setup and variables
  private DCMotor elevatorMotorModel =
      DCMotor.getNeoVortex(SimulationRobotConstants.SIM_MOTOR_COUNT);

  private SparkFlexSim elevatorMotorSim;
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.ELEVATOR_GEARING,
          SimulationRobotConstants.CARRIAGE_MASS.in(Kilogram),
          SimulationRobotConstants.ELEVATOR_DRUM_RADIUS.in(Meter),
          SimulationRobotConstants.MIN_ELEVATOR_HEIGHT.in(Meter),
          SimulationRobotConstants.MAX_ELEVATOR_HEIGHT.in(Meter),
          true,
          SimulationRobotConstants.MIN_ELEVATOR_HEIGHT.in(Meter));

  private DCMotor armMotorModel = DCMotor.getNeoVortex(SimulationRobotConstants.SIM_MOTOR_COUNT);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.ARM_REDUCTION,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.ARM_LENGTH.in(Meter),
              SimulationRobotConstants.ARM_MASS.in(Kilogram)),
          SimulationRobotConstants.ARM_LENGTH.in(Meter),
          SimulationRobotConstants.MIN_ANGLE_RADS,
          SimulationRobotConstants.MAX_ANGLE_RADS,
          true,
          SimulationRobotConstants.MIN_ANGLE_RADS);

  // Mechanism2d setup for subsystem
  private final Mechanism2d mech2d =
      new Mechanism2d(
          SimulationRobotConstants.MECH2D_WIDTH, SimulationRobotConstants.MECH2D_HEIGHT);
  private final MechanismRoot2d mech2dRoot =
      mech2d.getRoot(
          "Elevator Root",
          SimulationRobotConstants.ELEVATOR_MECH2D_X,
          SimulationRobotConstants.ELEVATOR_MECH2D_Y);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.MIN_ELEVATOR_HEIGHT.in(Meter)
                  * SimulationRobotConstants.PIXELS_PER_METER,
              ElevatorConstants.ELEVATOR_SIM_ANGLE.in(Degrees)));
  private final MechanismLigament2d armMech2d =
      elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.ARM_LENGTH.in(Meter)
                  * SimulationRobotConstants.PIXELS_PER_METER,
              ArmConstants.ARM_STARTING_ANGLE.in(Degrees)
                  - Units.radiansToDegrees(SimulationRobotConstants.MIN_ANGLE_RADS)
                  - ArmConstants.ARM_ANGLE_COMPENSATE.in(Degrees)));

  /** Creates a new ElevatorArmSubsystem. */
  public ElevatorArmSubsystem() {

    elevatorMotor.configure(
        Configs.ElevatorArmConfig.ELEVATOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    armMotor.configure(
        Configs.ElevatorArmConfig.ARM_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Resets elevator encoder position to 0
    elevatorEncoder.setPosition(0);

    // Display mechanism2d
    SmartDashboard.putData("Elevator Subsystem", mech2d);

    // Initialize simulation values
    elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
    armMotorSim = new SparkFlexSim(armMotor, armMotorModel);
  }

  public boolean reachedSetpoint() {
    return Math.abs(elevatorEncoder.getPosition() - elevatorSetpoint)
            <= ElevatorConstants.ELEVATOR_TOLERANCE
        && Math.abs(armEncoder.getPosition() - armSetpoint) <= ArmConstants.ARM_TOLERANCE;
  }

  private double getArmAngleRadians() {
    return Units.rotationsToRadians(armEncoder.getPosition());
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. The elevator will use REV
   * MAXMotion position control which will allow for a smooth acceleration and deceleration of the
   * mechanisms' setpoints and the Arm will use PIDController and ArmFeedforward from WPILib.
   */
  private void moveToSetpoint() {
    double armFeedforwardVoltage =
        armFeedforward.calculate(
            pid.getSetpoint().position
                - Units.rotationsToRadians(ArmConstants.ARM_FEEDFORWARD_OFFSET),
            pid.getSetpoint().velocity);
    SmartDashboard.putNumber("Arm Feed Forward", armFeedforwardVoltage);

    double pidOutput = pid.calculate(getArmAngleRadians(), Units.rotationsToRadians(armSetpoint));

    armMotor.setVoltage(pidOutput + armFeedforwardVoltage);

    elevatorClosedLoopController.setReference(elevatorSetpoint, ControlType.kPosition);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpoint(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kHover:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_HOVER_SETPOINT;
              armSetpoint = ArmConstants.ARM_HOVER_SETPOINT;
              break;
            case kIntake:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_INTAKE_SETPOINT;
              armSetpoint = ArmConstants.ARM_INTAKE_SETPOINT;
              break;
            case kIdleSetpoint:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_IDLE_SETPOINT;
              armSetpoint = ArmConstants.ARM_IDLE_SETPOINT;
              break;
            case kL1:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L1_SETPOINT;
              armSetpoint = ArmConstants.ARM_L1_SETPOINT;
              break;
            case kL2:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L2_SETPOINT;
              armSetpoint = ArmConstants.ARM_L2_SETPOINT;
              break;
            case kL3:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L3_SETPOINT;
              armSetpoint = ArmConstants.ARM_L3_SETPOINT;
              break;
            case kL4:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L4_SETPOINT;
              armSetpoint = ArmConstants.ARM_L4_SETPOINT;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();
    // Display subsystem values
    SmartDashboard.putNumber("Arm Target Position", armSetpoint);
    SmartDashboard.putNumber("Arm Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Target Position", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());

    // Update mechanism2d
    elevatorMech2d.setLength(
        SimulationRobotConstants.PIXELS_PER_METER
                * SimulationRobotConstants.MIN_ELEVATOR_HEIGHT.in(Meter)
            + SimulationRobotConstants.PIXELS_PER_METER
                * (elevatorEncoder.getPosition() / SimulationRobotConstants.ELEVATOR_GEARING)
                * (SimulationRobotConstants.ELEVATOR_DRUM_RADIUS.in(Meter)
                    * SimulationRobotConstants.DRUM_CIRCUMFERENCE.in(Meter)
                    * Math.PI));
    armMech2d.setAngle(
        ElevatorConstants.ELEVATOR_STARTING_ANGLE.in(Degrees)
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.MIN_ANGLE_RADS)
                + Units.rotationsToDegrees(
                    armEncoder.getPosition() / SimulationRobotConstants.ARM_REDUCTION))
            - SimulationRobotConstants.ELEVATOR_ACCOUNT.in(
                Degrees) // subtract 90 degrees to account for the elevator
        );
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return elevatorSim.getCurrentDrawAmps() + armSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(SimulationRobotConstants.SIM_STANDARD_LOOP.in(Second));
    armSim.update(SimulationRobotConstants.SIM_UPDATE_TIME.in(Second));

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.ELEVATOR_DRUM_RADIUS.in(Meter)
                        * SimulationRobotConstants.DRUM_CIRCUMFERENCE.in(Meter)
                        * Math.PI))
                * SimulationRobotConstants.ELEVATOR_GEARING)
            * SimulationRobotConstants.ONE_MINUTE.in(Seconds),
        RobotController.getBatteryVoltage(),
        SimulationRobotConstants.SIM_UPDATE_TIME.in(Second));
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            armSim.getVelocityRadPerSec() * SimulationRobotConstants.ARM_REDUCTION),
        RobotController.getBatteryVoltage(),
        SimulationRobotConstants.SIM_UPDATE_TIME.in(Second));
  }
}
