package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {
  public static final class EndEffectorSubsystem {
    public static final SparkFlexConfig END_EFFECTOR_CONFIG = new SparkFlexConfig();

    static {
      END_EFFECTOR_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(
              (int) Constants.EndEffectorConstants.END_EFFECTOR_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class IntakeSubsystem {
    public static final SparkFlexConfig INTAKE_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class IntakePivotSubsystem {
    public static final SparkFlexConfig INTAKE_PIVOT_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_PIVOT_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT.in(Amp));
    }
  }
}
