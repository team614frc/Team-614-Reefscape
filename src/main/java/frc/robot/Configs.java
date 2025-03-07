package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ArmConstants;

public final class Configs {

  public static final class ElevatorConfig {
    public static final SparkFlexConfig ELEVATOR_CONFIG = new SparkFlexConfig();

    static {
      ELEVATOR_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class ArmConfig {
    public static final SparkFlexConfig ARM_CONFIG = new SparkFlexConfig();

    static {
      ARM_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) ArmConstants.ARM_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class EndEffectorConfig {
    public static final SparkFlexConfig END_EFFECTOR_CONFIG = new SparkFlexConfig();

    static {
      END_EFFECTOR_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit(
              (int) Constants.EndEffectorConstants.END_EFFECTOR_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class IntakeConfig {
    public static final SparkFlexConfig INTAKE_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class IntakePivotConfig {
    public static final SparkFlexConfig INTAKE_PIVOT_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_PIVOT_CONFIG
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class CanalConfig {
    public static final SparkFlexConfig CANAL_CONFIG = new SparkFlexConfig();

    static {
      CANAL_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.CanalConstants.CANAL_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class ClimberConfig {
    public static final SparkFlexConfig CLIMBER_CONFIG = new SparkFlexConfig();

    static {
      CLIMBER_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_CURRENT_LIMIT.in(Amp));
    }
  }
}
