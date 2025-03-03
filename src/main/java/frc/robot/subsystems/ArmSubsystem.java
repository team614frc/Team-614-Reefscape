// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  public enum ArmSetpoint {
    kArmIdle,
    kArmHover,
    kArmL2,
    kArmL3,
    kPushArm,
    kScoreL3Arm,
    kScoreL2Arm,
    kOuttakeArmAlgaeL2,
    kOuttakeArmAlgaeL3,
    kPukeArm;
  }

  // Arm Motor
  private SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
  private SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder();

  private final ProfiledPIDController armPid =
      new ProfiledPIDController(
          ArmConstants.kP,
          ArmConstants.kI,
          ArmConstants.kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(ArmConstants.ARM_MAX_VELOCITY),
              Units.rotationsToRadians(ArmConstants.ARM_MAX_ACCELERATION)));

  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  // Default Current Target
  private double armSetpoint = ArmConstants.ARM_START_SETPOINT;

  /** Creates a new ElevatorArmSubsystem. */
  public ArmSubsystem() {

    armMotor.configure(
        Configs.ArmConfig.ARM_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean reachedSetpoint() {
    return Math.abs(armEncoder.getPosition() - armSetpoint) <= ArmConstants.ARM_TOLERANCE;
  }

  private double getArmAngleRadians() {
    return Units.rotationsToRadians(armEncoder.getPosition());
  }

  public boolean checkArmL3() {
    boolean a = armSetpoint == ArmConstants.ARM_L3_SETPOINT;
    boolean b =
        Math.abs(armEncoder.getPosition() - ArmConstants.ARM_L3_SETPOINT)
            <= ArmConstants.ARM_TOLERANCE;

    return (a || b);
  }

  public boolean checkArmPuke() {
    return armSetpoint == ArmConstants.ARM_PUKE_SETPOINT
        || Math.abs(armEncoder.getPosition() - ArmConstants.ARM_PUKE_SETPOINT)
            <= ArmConstants.ARM_TOLERANCE;
  }

  public boolean checkArmHover() {
    boolean a = armSetpoint == ArmConstants.ARM_HOVER_SETPOINT;
    boolean b =
        Math.abs(armEncoder.getPosition() - ArmConstants.ARM_HOVER_SETPOINT)
            <= ArmConstants.ARM_TOLERANCE;

    return (a || b);
  }

  /**
   * Drive the arm motor to their respective setpoint. The Arm will use PIDController and
   * ArmFeedforward from WPILib.
   */
  private void moveToSetpoint() {
    double armFeedforwardVoltage =
        armFeedforward.calculate(
            armPid.getSetpoint().position
                - Units.rotationsToRadians(ArmConstants.ARM_FEEDFORWARD_OFFSET),
            armPid.getSetpoint().velocity);

    double armPidOutput =
        armPid.calculate(getArmAngleRadians(), Units.rotationsToRadians(armSetpoint));

    armMotor.setVoltage(armPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Arm FF", armFeedforwardVoltage);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpoint(ArmSetpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kArmIdle:
              armSetpoint = ArmConstants.ARM_IDLE_SETPOINT;
              break;
            case kArmHover:
              armSetpoint = ArmConstants.ARM_HOVER_SETPOINT;
              break;
            case kArmL2:
              armSetpoint = ArmConstants.ARM_L2_SETPOINT;
              break;
            case kArmL3:
              armSetpoint = ArmConstants.ARM_L3_SETPOINT;
              break;
            case kPushArm:
              armSetpoint = ArmConstants.ARM_PUSH_SETPOINT;
              break;
            case kScoreL3Arm:
              armSetpoint = ArmConstants.ARM_L3_SCORE_SETPOINT;
              break;
            case kScoreL2Arm:
              armSetpoint = ArmConstants.ARM_L2_SCORE_SETPOINT;
              break;
            case kOuttakeArmAlgaeL2:
              armSetpoint = ArmConstants.ARM_FEEDFORWARD_OFFSET;
              break;
            case kOuttakeArmAlgaeL3:
              armSetpoint = ArmConstants.ARM_PUSH_SETPOINT;
              break;
            case kPukeArm:
              armSetpoint = ArmConstants.ARM_PUKE_SETPOINT;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();

    SmartDashboard.putNumber("Arm Target Position", armSetpoint);
    SmartDashboard.putNumber("Arm Actual Position", armEncoder.getPosition());
  }
}
