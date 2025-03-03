// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  public enum ElevatorSetpoint {
    kElevatorIdle,
    kElevatorHover,
    kElevatorL2,
    kElevatorL3,
    kElevatorOuttake,
    kOuttakeElevatorAlgae,
    kElevatorIntake;
  }

  // Elevator Motor
  private SparkFlex elevatorMotor =
      new SparkFlex(ElevatorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getExternalEncoder();

  private final ProfiledPIDController elevatorPid =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.ELEVATOR_MAX_VELOCITY,
              ElevatorConstants.ELEVATOR_MAX_ACCELERATION));

  // Default Current Target
  private double elevatorSetpoint = ElevatorConstants.ELEVATOR_IDLE_SETPOINT;

  /** Creates a new ElevatorArmSubsystem. */
  public ElevatorSubsystem() {

    elevatorMotor.configure(
        Configs.ElevatorConfig.ELEVATOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    elevatorEncoder.setPosition(0);
  }

  public boolean reachedSetpoint() {
    return Math.abs(elevatorEncoder.getPosition() - elevatorSetpoint)
        <= ElevatorConstants.ELEVATOR_TOLERANCE;
  }

  private double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command setElevatorResetSpeed() {
    return Commands.runEnd(
        () -> {
          elevatorMotor.set(ElevatorConstants.ELEVATOR_SLOW_DOWN_SPEED);
        },
        () -> {
          elevatorMotor.set(ElevatorConstants.ELEVATOR_STOP_SPEED);
        });
  }

  public boolean elevatorStalled() {
    return Math.abs(elevatorEncoder.getVelocity()) < ElevatorConstants.ELEVATOR_STALL_VELOCITY
        && elevatorMotor.get() == ElevatorConstants.ELEVATOR_SLOW_DOWN_SPEED;
  }

  public boolean checkElevatorL3() {
    boolean c = elevatorSetpoint == ElevatorConstants.ELEVATOR_L3_SETPOINT;
    boolean d =
        Math.abs(elevatorEncoder.getPosition() - ElevatorConstants.ELEVATOR_L3_SETPOINT)
            <= ElevatorConstants.ELEVATOR_TOLERANCE;

    return (c || d);
  }

  public boolean checkElevatorHover() {
    boolean c = elevatorSetpoint == ElevatorConstants.ELEVATOR_HOVER_SETPOINT;
    boolean d =
        Math.abs(elevatorEncoder.getPosition() - ElevatorConstants.ELEVATOR_HOVER_SETPOINT)
            <= ElevatorConstants.ELEVATOR_TOLERANCE;

    return (c || d);
  }

  public Command resetElevatorEncoder() {
    return Commands.runOnce(
        () -> {
          elevatorEncoder.setPosition(0);
        });
  }

  /**
   * Drive the elevator motor to their respective setpoint. The elevator will use raw PIDController
   * from WPILib.
   */
  private void moveToSetpoint() {
    double elevatorPidOutput = elevatorPid.calculate(getElevatorPosition(), elevatorSetpoint);
    elevatorMotor.setVoltage(elevatorPidOutput); // + ElevatorConstants.kG);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpoint(ElevatorSetpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kElevatorIdle:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_IDLE_SETPOINT;
              break;
            case kElevatorHover:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_HOVER_SETPOINT;
            case kElevatorL2:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L2_SETPOINT;
              break;
            case kElevatorL3:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L3_SETPOINT;
              break;
            case kElevatorOuttake:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_OUTTAKE_SETPOINT;
              break;
            case kOuttakeElevatorAlgae:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_L3_SETPOINT;
              break;
            case kElevatorIntake:
              elevatorSetpoint = ElevatorConstants.ELEVATOR_INTAKE_SETPOINT;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();

    SmartDashboard.putNumber("Elevator Target Position", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator Stalled", elevatorStalled());
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());
  }
}
