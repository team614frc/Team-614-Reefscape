// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mass ROBOT_MASS = Pounds.of(60);
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS.in(Kilogram));
  public static final Time LOOP_TIME = Seconds.of(0.13); // s, 20ms + 110ms sprk max velocity lag
  public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(14.5);

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled.
    public static final Time WHEEL_LOCK_TIME = Seconds.of(10);
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR = 22;
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final double ELEVATOR_P_VALUE = 0.0015;
    public static final double ELEVATOR_D_VALUE = 0.0035;
    public static final int ELEVATOR_FEEDER_SETPOINT = 0;
    public static final int ELEVATOR_LEVEL1_SETPOINT = 0;
    public static final int ELEVATOR_LEVEL2_SETPOINT = 15;
    public static final int ELEVATOR_LEVEL3_SETPOINT = 34;
    public static final int ELEVATOR_LEVEL4_SETPOINT = 0;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR = 24;
    public static final int INTAKE_PIVOT_MOTOR = 25;
    public static final int INTAKE_CURRENT_LIMIT = 40;
    public static final double OUTTAKE_SPEED = 0;
    public static final double INTAKE_SPEED = 0;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
  }

  public static final class CanalConstants {
    public static final int CANAL_MOTOR = 23;
    public static final int CANAL_CURRENT_LIMIT = 40;
    public static final double OUTTAKE_SPEED = 1;
    public static final double INTAKE_SPEED = -1;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR = 21;
    public static final int ARM_CURRENT_LIMIT = 40;
    public static final double ARM_P_VALUE = 0.005;
    public static final double ARM_D_VALUE = 0.005;
    public static final int ARM_FEEDER_SETPOINT = 0;
    public static final int ARM_LEVEL1_SETPOINT = 0;
    public static final int ARM_LEVEL2_SETPOINT = 5;
    public static final int ARM_LEVEL3_SETPOINT = 20;
    public static final int ARM_LEVEL4_SETPOINT = 0;
  }

  public static class EndEffectorConstants {
    public static final int END_EFFECTOR_CURRENT_LIMIT = 40;
    public static final double OUTTAKE_SPEED = 1;
    public static final double INTAKE_SPEED = -1;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
    public static final int END_EFFECTOR_MOTOR = 25;
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 12; // 12:1
    public static final double kCarriageMass = 2.72155 + 6.123497; // Kg, arm + elevator stage
    public static final double kElevatorDrumRadius = 0.0205232; // m
    public static final double kMinElevatorHeightMeters = 0.45085; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m
    public static final double kArmReduction = 30; // 30:1
    public static final double kArmLength = 0.52705; // m
    public static final double kArmMass = 2.72155; // Kg
    public static final double kMinAngleRads = Units.degreesToRadians(-90);
    public static final double kMaxAngleRads = Units.degreesToRadians(90); // from horiz
  }

  public static class OperatorConstants {
    // Joystick deadband.
    public static final double DEADBAND = 0.1;
  }
}
