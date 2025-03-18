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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase {
  private final SparkFlex intakePivotMotorRight =
      new SparkFlex(IntakeConstants.RIGHT_INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder intakePivotEncoderRight = intakePivotMotorRight.getEncoder();
  private final SparkFlex intakePivotMotorLeft =
      new SparkFlex(IntakeConstants.LEFT_INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder intakePivotEncoderLeft = intakePivotMotorLeft.getEncoder();

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          IntakeConstants.PIVOT_kP,
          IntakeConstants.PIVOT_kI,
          IntakeConstants.PIVOT_kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(IntakeConstants.PIVOT_MAX_VELOCITY),
              Units.rotationsToRadians(IntakeConstants.PIVOT_MAX_ACCELERATION)));

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          IntakeConstants.PIVOT_kS,
          IntakeConstants.PIVOT_kG,
          IntakeConstants.PIVOT_kV,
          IntakeConstants.PIVOT_kA);

  private double pivotSetpoint = IntakeConstants.PIVOT_OUTTAKE_ALGAE;

  public IntakePivotSubsystem() {
    intakePivotMotorLeft.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG_LEFT,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakePivotMotorRight.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG_RIGHT,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakePivotEncoderRight.setPosition(0);
    intakePivotEncoderLeft.setPosition(0);
  }

  public boolean reachedSetpoint() {
    return Math.abs(intakePivotEncoderLeft.getPosition() - pivotSetpoint)
        <= IntakeConstants.PIVOT_TOLERANCE;
  }

  private double getPivotAngleRadians() {
    return Units.rotationsToRadians(intakePivotEncoderLeft.getPosition());
  }

  private void pivotPIDControl() {

    double delta =
        Units.radiansToRotations(pid.getSetpoint().position)
            - IntakeConstants.PIVOT_FEEDFORWARD_OFFSET;
    delta /= IntakeConstants.PIVOT_GEAR_RATIO;

    double armFeedforwardVoltage =
        feedforward.calculate(Units.rotationsToRadians(delta), pid.getSetpoint().velocity);

    double pivotPidOutput =
        pid.calculate(getPivotAngleRadians(), Units.rotationsToRadians(pivotSetpoint));

    intakePivotMotorLeft.setVoltage(pivotPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Pivot FF", armFeedforwardVoltage);
  }

  @Override
  public void periodic() {
    pivotPIDControl();
    SmartDashboard.putData(pid);
    SmartDashboard.putNumber("Left Pivot Goal", pid.getGoal().position);
    SmartDashboard.putNumber("Left Pivot Position", intakePivotEncoderLeft.getPosition());
    SmartDashboard.putNumber("Right Pivot Position", intakePivotEncoderRight.getPosition());
  }

  public Command pivotDown() {
    return this.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_DOWN;
        });
  }

  public Command pivotIdle() {
    return this.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_UP;
        });
  }

  public Command pivotIntakeAlgae() {
    return this.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_INTAKE_ALGAE;
        });
  }

  public Command pivotOuttakeAlgae() {
    return this.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_OUTTAKE_ALGAE;
        });
  }
}
