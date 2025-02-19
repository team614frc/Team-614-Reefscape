// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanalConstants;

public class CanalSubsystem extends SubsystemBase {
  private final SparkFlex canalMotor =
      new SparkFlex(CanalConstants.CANAL_MOTOR, MotorType.kBrushless);

  private final LaserCan laserCan = new LaserCan(5);
  private final LaserCan.Measurement distance = laserCan.getMeasurement();

  /** Creates a new CanalSubsystem. */
  public CanalSubsystem() {
    canalMotor.configure(
        Configs.CanalConfig.CANAL_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    SmartDashboard.putNumber("Canal Motor Output", canalMotor.get());
    SmartDashboard.putBoolean("Game Piece Detection", gamePieceDetected());

    if (!RobotBase.isSimulation()) {
      SmartDashboard.putNumber("LaserCAN Distance", distance.distance_mm);
    }
  }

  public void set(double speed) {
    canalMotor.set(speed);
  }

  public SparkFlex getMotor() {
    return canalMotor;
  }

  public Command intake() {
    return Commands.runOnce(
        () -> {
          set(CanalConstants.INTAKE_SPEED);
        });
  }

  public Command outtake() {
    return Commands.runEnd(
        () -> {
          set(CanalConstants.OUTTAKE_SPEED);
        },
        () -> {
          set(CanalConstants.OUTTAKE_REST_SPEED);
        });
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          set(CanalConstants.CANAL_REST_SPEED);
        });
  }

  public boolean gamePieceDetected() {
    return (distance != null
        && distance.distance_mm < 185
        && distance.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
  }
}
