// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanalConstants;

public class CanalSubsystem extends SubsystemBase {
  private final SparkFlex canalMotor =
      new SparkFlex(CanalConstants.CANAL_MOTOR, MotorType.kBrushless);

  /** Creates a new CanalSubsystem. */
  public CanalSubsystem() {
    canalMotor.configure(
        Configs.CanalSubsystem.CANAL_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    SmartDashboard.putNumber("Canal Motor Output", canalMotor.get());
  }

  public void set(double speed) {
    canalMotor.set(speed);
  }

  public Command intakeCoralCanal() {
    return Commands.runEnd(
        () -> {
          set(CanalConstants.INTAKE_SPEED);
        },
        () -> {
          set(CanalConstants.INTAKE_REST_SPEED);
        });
  }

  public Command outtakeCoralCanal() {
    return Commands.runEnd(
        () -> {
          set(CanalConstants.OUTTAKE_SPEED);
        },
        () -> {
          set(CanalConstants.OUTTAKE_REST_SPEED);
        });
  }
}
