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
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private final SparkFlex intakeMotor =
      new SparkFlex(Constants.IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.configure(
        Configs.IntakeSubsystem.INTAKE_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getSpeed() {
    return intakeMotor.get();
  }

  @Override
  public void close() throws Exception {
    intakeMotor.close();
  }

  public void set() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed (RPM)", intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Motor Output", intakeMotor.get());
  }

  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public SparkFlex getMotor() {
    return intakeMotor;
  }

  public Command intakeGamepiece() {
    return Commands.run(() -> set(Constants.IntakeConstants.INTAKE_SPEED));
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

  public Command stopIntake() {
    return Commands.runOnce(() -> set(Constants.IntakeConstants.INTAKE_REST_SPEED));
  }
}
