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
  private final SparkFlex intakePivotMotor =
      new SparkFlex(IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder intakePivotEncoder = intakePivotMotor.getEncoder();

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

  private double pivotSetpoint = IntakeConstants.PIVOT_UP;

  public IntakePivotSubsystem() {
    intakePivotMotor.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakePivotEncoder.setPosition(0);
  }

  private double getPivotAngleRadians() {
    return Units.rotationsToRadians(intakePivotEncoder.getPosition());
  }

  private void pivotPIDControl() {
    double armFeedforwardVoltage =
        feedforward.calculate(
            (Units.radiansToRotations(pid.getSetpoint().position)
                    - IntakeConstants.PIVOT_FEEDFORWARD_OFFSET)
                * 0.254,
            pid.getSetpoint().velocity);

    double pivotPidOutput =
        pid.calculate(getPivotAngleRadians(), Units.rotationsToRadians(pivotSetpoint));

    intakePivotMotor.setVoltage(pivotPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Pivot FF", armFeedforwardVoltage);
  }

  @Override
  public void periodic() {
    pivotPIDControl();

    SmartDashboard.putData(pid);
    SmartDashboard.putNumber("Pivot Setpoint", pid.getSetpoint().position);
    SmartDashboard.putNumber("Pivot Goal", pid.getGoal().position);
    SmartDashboard.putNumber("Pivot Position", intakePivotEncoder.getPosition());
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

  public Command pivotAlgae() {
    return Commands.runOnce(
        () -> {
          pivotSetpoint = IntakeConstants.PIVOT_ALGAE;
        });
  }
}
