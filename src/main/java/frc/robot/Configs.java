package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ArmConstants;

public final class Configs {

  public static final class ElevatorArmSubsystem {
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig armConfig = new SparkFlexConfig();

    static {
      // Basic settings of elevator motor
      elevatorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(
              Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT); // .voltageCompensation(12);

      armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40); // .voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      armConfig
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

      elevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      elevatorConfig
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
    public static final SparkFlexConfig endEffectorConfig = new SparkFlexConfig();

    static {
      endEffectorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(Constants.EndEffectorConstants.END_EFFECTOR_CURRENT_LIMIT);
    }
  }

  public static final class CanalSubsystem {
    public static final SparkFlexConfig canalConfig = new SparkFlexConfig();

    static {
      canalConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(Constants.CanalConstants.CANAL_CURRENT_LIMIT);
    }
  }

  public static final class IntakeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig intakePivotConfig = new SparkFlexConfig();

    static {
      intakeConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT);

      intakePivotConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT);

      intakePivotConfig
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
}
