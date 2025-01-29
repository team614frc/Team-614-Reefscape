// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex intakeMotor =
      new SparkFlex(Constants.IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
  private final SparkFlex intakePivotMotor =
      new SparkFlex(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.configure(
        Configs.EndEffectorSubsystem.endEffectorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakePivotMotor.configure
        (Configs.IntakeSubsystem.intakePivotConfig, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public Command intakeGamepiece() {
    return Commands.runEnd(
        () -> {
          set(Constants.IntakeConstants.INTAKE_SPEED);
        },
        () -> {
          set(Constants.IntakeConstants.INTAKE_REST_SPEED);
        });
  }

  public Command pukeGamepiece() {
    return Commands.runEnd(
        () -> {
          set(Constants.IntakeConstants.OUTTAKE_SPEED);
        },
        () -> {
          set(Constants.IntakeConstants.OUTTAKE_REST_SPEED);
        });
  }
}