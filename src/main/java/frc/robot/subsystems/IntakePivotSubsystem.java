// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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

  public IntakePivotSubsystem() {
    intakePivotMotor.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakePivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {}

  private double getArmAngleRadians() {
    return Units.rotationsToRadians(intakePivotEncoder.getPosition());
  }

  public Command pivotPIDControl() {
    return run(
        () -> {
          double armFeedforwardVoltage =
              feedforward.calculate(
                  pid.getSetpoint().position
                      - Units.rotationsToRadians(IntakeConstants.PIVOT_FEEDFORWARD_OFFSET),
                  pid.getSetpoint().velocity);
          SmartDashboard.putNumber("Intake Pivot Feedforward Feed Forward", armFeedforwardVoltage);

          double pidOutput = pid.calculate(getArmAngleRadians(), pid.getGoal());

          intakePivotMotor.setVoltage(pidOutput + armFeedforwardVoltage);
        });
  }

  public Command pivotDown() {
    return Commands.runOnce(
        () -> {
          pid.setGoal(IntakeConstants.PIVOT_MIN.in(Degrees));
        });
  }

  public Command pivotUp() {
    return Commands.runOnce(
        () -> {
          pid.setGoal(IntakeConstants.PIVOT_MAX.in(Degrees));
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
