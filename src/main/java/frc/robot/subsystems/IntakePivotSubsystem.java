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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase {
  private final SparkFlex intakePivotMotor =
      new SparkFlex(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

  private final PIDController pidController =
      new PIDController(
          IntakeConstants.PIVOT_kP, IntakeConstants.PIVOT_kI, IntakeConstants.PIVOT_kD);

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
  }

  @Override
  public void periodic() {
    double output = pidController.calculate(getMeasurement());
    useOutput(output);
  }

  public void useOutput(double output) {
    double feed =
        feedforward.calculate(
            getPosition().in(Radians), IntakeConstants.VELOCITY_COMPONENT); // No velocity component
    intakePivotMotor.set(output + feed);
    SmartDashboard.putNumber("Pivot PID output", output);
  }

  public boolean atGoal(Angle goal, Angle threshold) {
    return Math.abs(getMeasurement() - goal.in(Radians)) < threshold.in(Radians);
  }

  public void set(double speed) {
    intakePivotMotor.set(speed);
  }

  public Command pivotDown() {
    return Commands.runOnce(
        () -> {
          pidController.setSetpoint(IntakeConstants.PIVOT_MIN.in(Degrees));
        });
  }

  public Command pivotUp() {
    return Commands.runOnce(
        () -> {
          pidController.setSetpoint(IntakeConstants.PIVOT_MAX.in(Degrees));
        });
  }

  public double getMeasurement() {
    return getPosition().in(Degrees);
  }

  public Angle getPosition() {
    var position = intakePivotMotor.getEncoder().getPosition();
    return Degree.of(
        position / IntakeConstants.GEAR_RATIO * IntakeConstants.FULL_CIRCLE.in(Degrees));
  }
}
