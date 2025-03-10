// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase {
  private final SparkFlex intakePivotMotorRight =
      new SparkFlex(IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder intakePivotEncoderRight = intakePivotMotorRight.getEncoder();
  private final SparkFlex intakePivotMotorLeft =
      new SparkFlex(IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder intakePivotEncoderLeft = intakePivotMotorLeft.getEncoder();

  private final ProfiledPIDController rightPid =
      new ProfiledPIDController(
          IntakeConstants.RIGHT_PIVOT_kP,
          IntakeConstants.RIGHT_PIVOT_kI,
          IntakeConstants.RIGHT_PIVOT_kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(IntakeConstants.PIVOT_MAX_VELOCITY),
              Units.rotationsToRadians(IntakeConstants.PIVOT_MAX_ACCELERATION)));

  private final ArmFeedforward rightFeedForward =
      new ArmFeedforward(
          IntakeConstants.RIGHT_PIVOT_kS,
          IntakeConstants.RIGHT_PIVOT_kG,
          IntakeConstants.RIGHT_PIVOT_kV,
          IntakeConstants.RIGHT_PIVOT_kA);

  private final ProfiledPIDController leftPid =
      new ProfiledPIDController(
          IntakeConstants.LEFT_PIVOT_kP,
          IntakeConstants.LEFT_PIVOT_kI,
          IntakeConstants.LEFT_PIVOT_kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(IntakeConstants.PIVOT_MAX_VELOCITY),
              Units.rotationsToRadians(IntakeConstants.PIVOT_MAX_ACCELERATION)));

  private final ArmFeedforward leftFeedForward =
      new ArmFeedforward(
          IntakeConstants.LEFT_PIVOT_kS,
          IntakeConstants.LEFT_PIVOT_kG,
          IntakeConstants.LEFT_PIVOT_kV,
          IntakeConstants.LEFT_PIVOT_kA);

  private double pivotSetpoint = IntakeConstants.PIVOT_OUTTAKE_ALGAE;

  public IntakePivotSubsystem() {
    intakePivotMotorRight.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakePivotMotorLeft.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakePivotEncoderRight.setPosition(0);
    intakePivotEncoderLeft.setPosition(0);
  }

  public boolean reachedSetpoint() {
    return Math.abs(intakePivotEncoderLeft.getPosition() - pivotSetpoint)
            <= IntakeConstants.PIVOT_TOLERANCE
        && Math.abs(intakePivotEncoderRight.getPosition() - pivotSetpoint)
            <= IntakeConstants.PIVOT_TOLERANCE;
  }

  private double getPivotAngleRadians(RelativeEncoder enconder) {
    return Units.rotationsToRadians(enconder.getPosition());
  }

  private void pivotPIDControl(
      ProfiledPIDController pid,
      SparkFlex motor,
      ArmFeedforward feedforward,
      RelativeEncoder encoder) {
    double armFeedforwardVoltage =
        feedforward.calculate(
            (Units.radiansToRotations(pid.getSetpoint().position)
                    - IntakeConstants.PIVOT_FEEDFORWARD_OFFSET)
                * 0.254,
            pid.getSetpoint().velocity);

    double pivotPidOutput =
        pid.calculate(getPivotAngleRadians(encoder), Units.rotationsToRadians(pivotSetpoint));

    motor.setVoltage(pivotPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Pivot FF", armFeedforwardVoltage);
  }

  @Override
  public void periodic() {
    pivotPIDControl(leftPid, intakePivotMotorLeft, leftFeedForward, intakePivotEncoderLeft);
    pivotPIDControl(rightPid, intakePivotMotorRight, rightFeedForward, intakePivotEncoderRight);
    SmartDashboard.putData(leftPid);
    SmartDashboard.putData(rightPid);
    SmartDashboard.putNumber("Left Pivot Goal", leftPid.getGoal().position);
    SmartDashboard.putNumber("Left Pivot Position", intakePivotEncoderLeft.getPosition());
    SmartDashboard.putNumber("Right Pivot Goal", rightPid.getGoal().position);
    SmartDashboard.putNumber("Right Pivot Position", intakePivotEncoderRight.getPosition());
  }

  public Command pivotDown() {
    return Commands.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_DOWN;
        });
  }

  public Command pivotIdle() {
    return Commands.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_UP;
        });
  }

  public Command pivotIntakeAlgae() {
    return Commands.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_INTAKE_ALGAE;
        });
  }

  public Command pivotOuttakeAlgae() {
    return Commands.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_OUTTAKE_ALGAE;
        });
  }
}
