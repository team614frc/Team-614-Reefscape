// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.Constants.IntakePivotSetpoint;

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
              IntakeConstants.PIVOT_MAX_VELOCITY.in(RadiansPerSecond),
              IntakeConstants.PIVOT_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          IntakeConstants.PIVOT_kS,
          IntakeConstants.PIVOT_kG,
          IntakeConstants.PIVOT_kV,
          IntakeConstants.PIVOT_kA);

  private IntakePivotSetpoint pivotSetpoint = IntakePivotSetpoint.OUTTAKE_ALGAE;

  public IntakePivotSubsystem() {
    intakePivotMotor.configure(
        Configs.IntakePivotConfig.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakePivotEncoder.setPosition(0);
  }

  public boolean atSetpoint() {
    return Math.abs(intakePivotEncoder.getPosition() - pivotSetpoint.value.in(Rotations))
        <= IntakeConstants.PIVOT_TOLERANCE.in(Rotations);
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
        pid.calculate(getPivotAngleRadians(), pivotSetpoint.value.in(Rotations));

    intakePivotMotor.setVoltage(pivotPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Pivot FF", armFeedforwardVoltage);
  }

  /** Set the pivot to a predefined setpoint. */
  public Command setSetpoint(IntakePivotSetpoint setpoint) {
    return this.runOnce(() -> pivotSetpoint = setpoint);
  }

  @Override
  public void periodic() {
    pivotPIDControl();

    SmartDashboard.putData(pid);
    SmartDashboard.putNumber("Pivot Goal", pid.getGoal().position);
    SmartDashboard.putNumber("Pivot Position", intakePivotEncoder.getPosition());
  }
}
