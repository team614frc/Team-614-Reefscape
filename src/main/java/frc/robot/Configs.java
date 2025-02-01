package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ArmConstants;

public final class Configs {

  public static final class ElevatorArmSubsystem {
    public static final SparkFlexConfig ELEVATOR_CONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig ARM_CONFIG = new SparkFlexConfig();

    static {
      // Basic settings of elevator motor
      ELEVATOR_CONFIG
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(
              (int)
                  Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT.in(
                      Amp)); // .voltageCompensation(12);

      ARM_CONFIG.idleMode(IdleMode.kCoast).smartCurrentLimit(40); // .voltageCompensation(12);

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
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(5250)
          .maxAcceleration(7500)
          .allowedClosedLoopError(0.25);

      ELEVATOR_CONFIG
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      ELEVATOR_CONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // PID Values for position control
          .p(Constants.ElevatorConstants.ELEVATOR_P_VALUE)
          .d(Constants.ElevatorConstants.ELEVATOR_D_VALUE)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(5000)
          .maxAcceleration(7500)
          .allowedClosedLoopError(0.5);
    }
  }

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
