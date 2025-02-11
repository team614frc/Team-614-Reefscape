package frc.robot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public final class Configs {

  public static final class ElevatorArmConfig {
    public static final SparkFlexConfig ELEVATOR_CONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig ARM_CONFIG = new SparkFlexConfig();

    static {
      // Basic settings of elevator motor
      ELEVATOR_CONFIG
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(
              (int)
                  Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT.in(
                      Amp)); // .voltageCompensation(12);
      ELEVATOR_CONFIG
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);
      ELEVATOR_CONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          // PID Values for position control
          // .p(Constants.ElevatorConstants.ELEVATOR_P_VALUE)
          // .d(Constants.ElevatorConstants.ELEVATOR_D_VALUE)
          .pidf(
              ElevatorConstants.ELEVATOR_P_VALUE,
              0,
              ElevatorConstants.ELEVATOR_D_VALUE,
              ElevatorConstants.ELEVATOR_F_VALUE)
          .outputRange(ElevatorConstants.ELEVATOR_MIN_RANGE, ElevatorConstants.ELEVATOR_MAX_RANGE)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ElevatorConstants.ELEVATOR_MAX_VELOCITY.in(Rotations.per(Minute)))
          .maxAcceleration(
              ElevatorConstants.ELEVATOR_MAX_ACCELERATION.in(Rotations.per(Minute).per(Second)))
          .allowedClosedLoopError(ElevatorConstants.ELEVATOR_LOOP_ERROR);

      ELEVATOR_CONFIG.externalEncoder.inverted(true);

      ARM_CONFIG
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(
              (int) ArmConstants.ARM_CURRENT_LIMIT.in(Amp)); // .voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      ARM_CONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(ArmConstants.ARM_P_VALUE)
          .d(ArmConstants.ARM_D_VALUE)
          .outputRange(ArmConstants.ARM_MIN_RANGE, ArmConstants.ARM_MAX_RANGE)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ArmConstants.ARM_MAX_VELOCITY.in(Rotations.per(Minute)))
          .maxAcceleration(ArmConstants.ARM_MAX_ACCELERATION.in(Rotations.per(Minute).per(Second)))
          .allowedClosedLoopError(ArmConstants.ARM_LOOP_ERROR);
    }
  }

  public static final class EndEffectorConfig {
    public static final SparkFlexConfig END_EFFECTOR_CONFIG = new SparkFlexConfig();

    static {
      END_EFFECTOR_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(
              (int) Constants.EndEffectorConstants.END_EFFECTOR_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class IntakeConfig {
    public static final SparkFlexConfig INTAKE_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class IntakePivotConfig {
    public static final SparkFlexConfig INTAKE_PIVOT_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_PIVOT_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class CanalConfig {
    public static final SparkFlexConfig CANAL_CONFIG = new SparkFlexConfig();

    static {
      CANAL_CONFIG
          .idleMode(IdleMode.kCoast)
          .inverted(true)
          .smartCurrentLimit((int) Constants.CanalConstants.CANAL_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class ClimberConfig {
    public static final SparkFlexConfig CLIMBER_CONFIG = new SparkFlexConfig();

    static {
      CLIMBER_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_CURRENT_LIMIT.in(Amp));
    }
  }
}