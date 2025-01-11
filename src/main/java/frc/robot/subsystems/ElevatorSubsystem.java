// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // Creating a new motor
    SparkFlex leftElevatorMotor =
        new SparkFlex(ElevatorConstants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
    SparkFlexConfig leftElevatorConfig = new SparkFlexConfig();
    SparkFlex rightElevatorMotor =
        new SparkFlex(ElevatorConstants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);
    SparkFlexConfig rightElevatorConfig = new SparkFlexConfig();

    leftElevatorConfig
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast);

    // NEW: ResetMode and PersistMode are burnFlash() and restoreFactoryDefaults()
    leftElevatorMotor.configure(
        leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftElevatorMotor.close();

    rightElevatorConfig
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast);

    rightElevatorMotor.configure(
        rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevatorMotor.close();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
