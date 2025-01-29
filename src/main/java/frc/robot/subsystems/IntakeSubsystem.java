// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.util.Units;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex intakeMotor =
      new SparkFlex(Constants.IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
  private final SparkFlex intakePivotMotor =
      new SparkFlex(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          IntakeConstants.PIVOT_kS,
          IntakeConstants.PIVOT_kG,
          IntakeConstants.PIVOT_kV,
          IntakeConstants.PIVOT_kA);

  public IntakeSubsystem() {
    intakeMotor.configure(
        Configs.EndEffectorSubsystem.endEffectorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakePivotMotor.configure(
        Configs.IntakeSubsystem.intakePivotConfig,
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

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feed = feedforward.calculate(setpoint.position, setpoint.velocity);
    intakePivotMotor.set(output + 0.03); // + feed F = 0.01
    SmartDashboard.putNumber("Pivot PID output", output);
  }

  public boolean atGoal(Angle goal, Angle threshold) {
    return Math.abs(getMeasurement() - goal.in(Radians)) < threshold.in(Radians);
  }

  public void setPivotSpeed(double speed) {
    intakePivotMotor.set(speed);
  }

  public Command pivotDown() {
    return Commands.runOnce(
        () -> {
          setGoal(IntakeConstants.PIVOT_MIN.in(Degrees));
          enable();
          SmartDashboard.putNumber("Encoder Position in Command", getPosition().in(Degrees));
        });
  }

  public Command pivotUp() {
    return Commands.runOnce(
        () -> {
          setGoal(IntakeConstants.PIVOT_MAX.in(Degrees));
          enable();
          SmartDashboard.putNumber("Pivot Position (Degrees)", getPosition().in(Degrees));
        });
  }

  public double getMeasurement() {
    return getPosition().in(Degrees);
  }

  public Angle getPosition() {
    var position = intakePivotMotor.getEncoder().getPosition();
    return Degree.of(position / IntakeConstants.GEAR_RATIO * 360);
  }
}
