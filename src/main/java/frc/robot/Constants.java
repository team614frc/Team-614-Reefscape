// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
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
  public static final Pose3d CAMERA_OFFSET =
      new Pose3d(
          new Translation3d(Meters.of(0), Inches.of(-9), Meters.of(.5)),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));
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
    public static final Boolean USE_LIMELIGHT_FRONT = true;
    public static final Boolean USE_LIMELIGHT_BACK = false;
    public static final String LIMELIGHT_FRONT_NAME = "limelight-front";
    public static final String LIMELIGHT_BACK_NAME = "limelight-back";
    public static final LinearVelocity MAX_ALIGNMENT_VELOCITY = MetersPerSecond.of(1.3);
    public static final LinearAcceleration MAX_ALIGNMENT_ACCELERATION =
        MetersPerSecondPerSecond.of(1);
    public static final AngularVelocity MAX_ALIGNMENT_ANGULAR_VELOCITY = DegreesPerSecond.of(45);
    public static final AngularAcceleration MAX_ALIGNMENT_ANGULAR_ACCELERATION =
        DegreesPerSecondPerSecond.of(45);
    public static final Distance ALIGNMENT_TOLERANCE = Meters.of(.25);
    public static final Distance ALIGNMENT_SHIFT_TOLERANCE = Meters.of(.6);
    public static final ChassisSpeeds CORAL_DRIVE_SPEED = new ChassisSpeeds(-1, 0, 0);
    public static final List<Double> APRIL_TAGS =
        List.of(17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0);

    public static enum DetectionMode {
      APRILTAG,
      CORAL
    }
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR = 24;
    public static final int LEFT_INTAKE_PIVOT_MOTOR = 28;
    public static final int RIGHT_INTAKE_PIVOT_MOTOR = 26;
    public static final Current INTAKE_CURRENT_LIMIT = Amp.of(80);
    public static final Current INTAKE_PIVOT_CURRENT_LIMIT = Amp.of(40);
    public static final double OUTTAKE_SPEED = -0.2;
    public static final double PASSTHROUGH_SPEED = 0.7;
    public static final double FAST_OUTTAKE_SPEED = -0.3;
    public static final double INTAKE_SPEED = 0.25;
    public static final double ALGAE_INTAKE_SPEED = -0.3;
    public static final double ALGAE_OUTTAKE_SPEED = 0.3;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
    public static final double PIVOT_GEAR_RATIO = 60;

    public static final double PIVOT_kP = 6.5;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;
    public static final double PIVOT_kS = 0;
    public static final double PIVOT_kG = 0.4;
    public static final double PIVOT_kV = 0;
    public static final double PIVOT_kA = 0;

    public static final double GEAR_RATIO = 60;
    public static final double PIVOT_UP = -1.5;
    public static final double PIVOT_DOWN = 0.31;
    public static final double RIGHT_PIVOT_DOWN = PIVOT_DOWN + 0.02;
    public static final double PIVOT_INTAKE_ALGAE = 0.195;
    public static final double PIVOT_OUTTAKE_ALGAE = 0.117;
    public static final double PIVOT_MOTOR_SPEED = 0.1;
    public static final double PIVOT_REST_SPEED = 0;
    public static final double PIVOT_MAX_VELOCITY = 2;
    public static final double PIVOT_MAX_ACCELERATION = 2;
    public static final double PIVOT_FEEDFORWARD_OFFSET = 0.32;
    public static final double PIVOT_TOLERANCE = 0.75;
  }

  public static class EndEffectorConstants {
    public static final int END_EFFECTOR_MOTOR = 25;
    public static final Current END_EFFECTOR_CURRENT_LIMIT = Amp.of(80);
    public static final double OUTTAKE_SPEED = -0.45;
    public static final double INTAKE_SPEED = 0.35;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_ALGAE = -0.75;
    public static final double END_EFFECTOR_MIN_OUTPUT = 0.075;
    public static final double STALL_SPEED = 0.1;
    public static final double MIN_CURRENT = 22.75;
  }

  public static final class CanalConstants {
    public static final int CANAL_MOTOR = 23;
    public static final Current CANAL_CURRENT_LIMIT = Amp.of(80);
    public static final double BACKWARDS_SPEED = -0.095;
    public static final double INTAKE_SPEED = 0.215;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
    public static final double CANAL_FAST_SPEED = 0.3;
    public static final int CANAL_REST_SPEED = 0;
    public static final double CANAL_SLOW_SPEED = 0.085;
    public static final int LASER_MAX_DISTANCE = 200;
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
    public static final double ELEVATOR_HOVER_SETPOINT = 4.25;
    public static final double ELEVATOR_INTAKE_SETPOINT = 3.5;
    public static final double ELEVATOR_IDLE_SETPOINT = 0.05;
    public static final double ELEVATOR_L1_SETPOINT = 0;
    public static final double ELEVATOR_L2_SETPOINT = 0.4;
    public static final double ELEVATOR_L2_ALGAE_SETPOINT = 5.35;
    public static final double ELEVATOR_L3_SETPOINT = 6.19;
    public static final double ELEVATOR_L4_SETPOINT = 0;
    public static final double ELEVATOR_OUTTAKE_SETPOINT = 1.75;
    public static final int ELEVATOR_ZERO_ENCODER = 0;
    public static final double ELEVATOR_MAX_VELOCITY = 23;
    public static final double ELEVATOR_MAX_ACCELERATION = 50;
    public static final Angle ELEVATOR_SIM_ANGLE = Degrees.of(90);
    public static final Angle ELEVATOR_SIM_STARTING_ANGLE = Degrees.of(180);
    public static final double ELEVATOR_TOLERANCE = 0.1;
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
    public static final double ARM_HOVER_SETPOINT = 0.03;
    public static final double ARM_INTAKE_SETPOINT = 0.0285;
    public static final double ARM_PUSH_SETPOINT = 0.465;
    public static final double ARM_IDLE_SETPOINT = 0.49;
    public static final double ARM_START_SETPOINT = 0.5;
    public static final double ARM_L2_SETPOINT = 0.465;
    public static final double ARM_L3_SETPOINT = 0.475;
    public static final double ARM_L4_SETPOINT = 0;
    public static final double ARM_L3_SCORE_SETPOINT = 0.385;
    public static final double ARM_L2_SCORE_SETPOINT = 0.371;
    public static final double ARM_FEEDFORWARD_OFFSET = 0.278;
    public static final double ARM_PUKE_SETPOINT = 0.175;
    public static final int ARM_ZERO_ENCODER = 0;
    public static final double ARM_MAX_VELOCITY = 4;
    public static final double ARM_MAX_ACCELERATION = 1.25;
    public static final double ARM_MIN_RANGE = -1;
    public static final double ARM_MAX_RANGE = 1;
    public static final double ARM_LOOP_ERROR = 0.25;
    public static final Angle ARM_STARTING_ANGLE = Degrees.of(180);
    public static final Angle ARM_ANGLE_COMPENSATE = Degrees.of(90);
    public static final double ARM_TOLERANCE = 0.05;
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
    public static final double CLIMB_SPEED = 0.75;
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
