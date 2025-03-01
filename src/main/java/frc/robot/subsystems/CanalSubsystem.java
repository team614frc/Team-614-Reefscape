// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
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

  private final LaserCan laserCan = new LaserCan(5);
  private LaserCan.Measurement distance;

  /** Creates a new CanalSubsystem. */
  public CanalSubsystem() {
    canalMotor.configure(
        Configs.CanalConfig.CANAL_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize LaserCan settings
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    distance = laserCan.getMeasurement();

    if (distance != null) {
      SmartDashboard.putNumber("LaserCAN Distance", distance.distance_mm);
    } else {
      SmartDashboard.putString("LaserCAN Distance", "No Data");
    }

    SmartDashboard.putBoolean("Game Piece Detection", gamePieceDetected());
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

  public Command slow() {
    return Commands.runOnce(
        () -> {
          set(CanalConstants.CANAL_SLOW_SPEED);
        });
  }

  public Command fast() {
    return Commands.runOnce(
        () -> {
          set(CanalConstants.CANAL_FAST_SPEED);
        });
  }

  public Command backwards() {
    return Commands.runOnce(
        () -> {
          set(CanalConstants.BACKWARDS_SPEED);
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
        && distance.distance_mm < CanalConstants.LASER_MAX_DISTANCE
        && distance.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
  }
}
