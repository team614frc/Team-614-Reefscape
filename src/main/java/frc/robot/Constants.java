// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;
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
  public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(17.5);

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled.
    public static final Time WHEEL_LOCK_TIME = Seconds.of(10);
    public static final double AUTO_TRANSLATION_kP = 2;
    public static final double AUTO_TRANSLATION_kI = 0.0;
    public static final double AUTO_TRANSLATION_kD = 0.0;
    public static final double AUTO_ROTATION_kP = 6;
    public static final double AUTO_ROTATION_kI = 0.0;
    public static final double AUTO_ROTATION_kD = 0.0;
  }

  public enum IntakePivotSetpoint {
    UP(-1.5),
    DOWN(-7.2),
    INTAKE_ALGAE(-4.4),
    OUTTAKE_ALGAE(-2.6);

    public final Angle value;

    IntakePivotSetpoint(double value) {
      this.value = Rotations.of(value);
    }
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR = 24;
    public static final int INTAKE_PIVOT_MOTOR = 26;
    public static final Current INTAKE_CURRENT_LIMIT = Amp.of(80);
    public static final Current INTAKE_PIVOT_CURRENT_LIMIT = Amp.of(40);
    public static final double OUTTAKE_SPEED = -0.2;
    public static final double FAST_OUTTAKE_SPEED = -0.3;
    public static final double INTAKE_SPEED = 0.3;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;

    public static final double PIVOT_kP = 0.8;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0.01;
    public static final double PIVOT_kS = 0;
    public static final double PIVOT_kG = 0.8;
    public static final double PIVOT_kV = 0;
    public static final double PIVOT_kA = 0;
    public static final double GEAR_RATIO = 60;
    public static final double PIVOT_MOTOR_SPEED = 0.1;
    public static final double PIVOT_REST_SPEED = 0;
    public static final AngularVelocity PIVOT_MAX_VELOCITY = RotationsPerSecond.of(8);
    public static final AngularAcceleration PIVOT_MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(12);
    public static final double PIVOT_FEEDFORWARD_OFFSET = -6.3;
    public static final Angle PIVOT_TOLERANCE = Rotations.of(0.75);
  }

  public static class EndEffectorConstants {
    public static final Current END_EFFECTOR_CURRENT_LIMIT = Amp.of(80);
    public static final double OUTTAKE_SPEED = -0.35;
    public static final double INTAKE_SPEED = 0.125;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_ALGAE = -0.75;
    public static final int END_EFFECTOR_MOTOR = 25;
    public static final double END_EFFECTOR_MIN_OUTPUT = 0.15;
    public static final Angle END_EFFECTOR_MIN_RPM = Rotations.of(50);
    public static final double STALL_SPEED = 0.1;
  }

  public static final class CanalConstants {
    public static final int CANAL_MOTOR = 23;
    public static final Current CANAL_CURRENT_LIMIT = Amp.of(80);
    public static final double BACKWARDS_SPEED = -0.095;
    public static final double INTAKE_SPEED = 0.1;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
    public static final double CANAL_FAST_SPEED = 0.3;
    public static final int CANAL_REST_SPEED = 0;
    public static final double CANAL_SLOW_SPEED = 0.085;
    public static final int LASER_MAX_DISTANCE = 180;
  }

  public enum ElevatorSetpoint {
    HOVER(4.85),
    INTAKE_UP(5.5),
    INTAKE(3.74),
    IDLE(0.05),
    PREP_L2(0.4),
    PREP_L3(6.19),
    PREP_L4(0),
    OUTTAKE(1.75),
    PUNCH_ALGAE(6.19);

    public final Angle value;

    ElevatorSetpoint(double value) {
      this.value = Rotations.of(value);
    }
  }

  public enum ArmSetpoint {
    HOVER(0.03),
    INTAKEUP(0.015),
    PUSH(0.465),
    IDLE(0.49),
    START(0.5),
    PREP_L2(0.465),
    PREP_L3(0.475),
    PREP_L4(0),
    SCORE_L3(0.390),
    SCORE_L2(0.380),
    PUKE(0.175),
    PUNCH_ALGAE_L2(0.278),
    PUNCH_ALGAE_L3(0.465);

    public final Angle value;

    ArmSetpoint(double value) {
      this.value = Rotations.of(value);
    }
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR = 22;
    public static final Current ELEVATOR_CURRENT_LIMIT = Amp.of(80);
    public static final double kP = 6;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0.02;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final AngularVelocity ELEVATOR_MAX_VELOCITY = RotationsPerSecond.of(20);
    public static final AngularAcceleration ELEVATOR_MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(50);
    public static final Angle ELEVATOR_SIM_ANGLE = Degrees.of(90);
    public static final Angle ELEVATOR_SIM_STARTING_ANGLE = Degrees.of(180);
    public static final Angle ELEVATOR_TOLERANCE = Rotations.of(0.1);
    public static final double ELEVATOR_SLOW_DOWN_SPEED = -0.05;
    public static final double ELEVATOR_STALL_VELOCITY = 0.1;
    public static final double ELEVATOR_STOP_SPEED = 0;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR = 21;
    public static final Current ARM_CURRENT_LIMIT = Amp.of(80);
    public static final double kP = 4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0.85;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final Angle ARM_FEEDFORWARD_OFFSET = Rotations.of(0.278);
    public static final int ARM_ZERO_ENCODER = 0;
    public static final AngularVelocity ARM_MAX_VELOCITY = RotationsPerSecond.of(4);
    public static final AngularAcceleration ARM_MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(1);
    public static final double ARM_MIN_RANGE = -1;
    public static final double ARM_MAX_RANGE = 1;
    public static final double ARM_LOOP_ERROR = 0.25;
    public static final Angle ARM_STARTING_ANGLE = Degrees.of(180);
    public static final Angle ARM_ANGLE_COMPENSATE = Degrees.of(90);
    public static final Angle ARM_TOLERANCE = Rotations.of(0.05);
  }

  public static final class SimulationRobotConstants {
    public static final double PIXELS_PER_METER = 20;

    public static final double ELEVATOR_GEARING = 12; // 12:1
    public static final Mass CARRIAGE_MASS =
        Kilogram.of(2.72155 + 6.123497); // Kg, arm + elevator stage
    public static final Distance ELEVATOR_DRUM_RADIUS = Meter.of(0.0205232); // m
    public static final Distance MIN_ELEVATOR_HEIGHT = Meter.of(0.45085); // m
    public static final Distance MAX_ELEVATOR_HEIGHT = Meter.of(1.62); // m
    public static final double ARM_REDUCTION = 30; // 30:1
    public static final Distance ARM_LENGTH = Meter.of(0.52705); // m
    public static final Mass ARM_MASS = Kilogram.of(2.72155); // Kg
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(270);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(90); // from horiz
    public static final Time SIM_STANDARD_LOOP = Second.of(0.020);
    public static final int MECH2D_WIDTH = 50;
    public static final int MECH2D_HEIGHT = 50;
    public static final int ELEVATOR_MECH2D_X = 25;
    public static final int ELEVATOR_MECH2D_Y = 0;
    public static final int SIM_MOTOR_COUNT = 1;
    public static final Time SIM_UPDATE_TIME = Second.of(0.02);
    public static final Angle ELEVATOR_ACCOUNT = Degrees.of(90);
    public static final Time ONE_MINUTE = Seconds.of(60);
    public static final Distance DRUM_CIRCUMFERENCE = Meter.of(2.0);
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR = 27;
    public static final Current CLIMBER_CURRENT_LIMIT = Amp.of(80);
    public static final double CLIMB_SPEED = 0.5;
    public static final double REVERSE_CLIMB_SPEED = -0.5;
    public static final double CLIMB_REST_SPEED = 0;
  }

  public static class OperatorConstants {
    // Joystick deadband.
    public static final double DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int CODRIVER_PORT = 1;
    public static final double RUMBLE_REST = 0;
    public static final double RUMBLE_SPEED = 1;
    public static final double RUMBLE_DURATION = 0.25;
  }

  public static final class LEDConstants {
    public static final Color AUTO_COLOR = Color.kRed;
    public static final Color TELEOP_COLOR = Color.kOrange;
    public static final Color ENDGAME_COLOR = Color.kOrange;
    public static final Color ALIGNMENT_COLOR = Color.kOrange;
    public static final Distance SPACING = Meters.of(1 / 120.0);
    public static final LinearVelocity SCROLL_SPEED = MetersPerSecond.of(1);
    public static final Time BREATHE_TIME = Seconds.of(2);
  }
}
