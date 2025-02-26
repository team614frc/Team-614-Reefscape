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

  private double pivotSetpoint = IntakeConstants.PIVOT_UP;

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

  public IntakePivotSubsystem() {
    intakePivotMotor.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakePivotEncoder.setPosition(0);
  }

  private double getArmAngleRadians() {
    return intakePivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // pivotPIDControl();

    double armFeedforwardVoltage =
        feedforward.calculate(
            pid.getSetpoint().position
                - IntakeConstants.PIVOT_FEEDFORWARD_OFFSET,
            pid.getSetpoint().velocity);
    SmartDashboard.putNumber("Intake Pivot Feedforward Feed Forward", pivotSetpoint);

    double pidOutput = pid.calculate(getArmAngleRadians(), pivotSetpoint);

    intakePivotMotor.setVoltage(pidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Get Setpoint", pid.getSetpoint().position);
    SmartDashboard.putNumber("Pivot Goal", pid.getGoal().position);
    SmartDashboard.putNumber("Pivot Position", intakePivotEncoder.getPosition());
  }


  // private void pivotPIDControl() {
  //   // double armFeedforwardVoltage =
  //   //     feedforward.calculate(
  //   //         pid.getSetpoint().position
  //   //             - Units.rotationsToRadians(IntakeConstants.PIVOT_FEEDFORWARD_OFFSET),
  //   //         pid.getSetpoint().velocity);
  //   // SmartDashboard.putNumber("Intake Pivot Feedforward Feed Forward", pid.getGoal().position);

  //   double pidOutput = pid.calculate(getArmAngleRadians(), pid.getGoal());

  //   // intakePivotMotor.setVoltage(pidOutput + armFeedforwardVoltage);
  // }

  public Command pivotDown() {
    return Commands.runOnce(
        () -> {
          pid.setGoal(Units.rotationsToRadians(IntakeConstants.PIVOT_DOWN));
        });
  }

  public Command pivotIdle() {
    return Commands.runOnce(
        () -> {
          pid.setGoal(Units.rotationsToRadians(IntakeConstants.PIVOT_UP));
        });
  }

  public Command pivotAlgae() {
    return Commands.runOnce(
        () -> {
          pid.setGoal(Units.rotationsToRadians(IntakeConstants.PIVOT_ALGAE));
        });
  }
}
