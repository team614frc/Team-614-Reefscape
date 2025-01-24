package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ArmConstants;

public final class Configs {

  public static final class ElevatorSubsystem {
    public static final SparkFlexConfig rightElevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig leftElevatorConfig = new SparkFlexConfig();
    public static final SparkMaxConfig leftArmConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightArmConfig = new SparkMaxConfig();

    static {
      // Basic settings of elevator motor
      rightElevatorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(
              Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT); // .voltageCompensation(12);
      leftElevatorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(
              Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT); // .voltageCompensation(12);

      leftArmConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40); // .voltageCompensation(12);
      rightArmConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(40)
          .inverted(true); // .voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      leftArmConfig
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

      rightArmConfig
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

      leftElevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);
      rightElevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      leftElevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // PID Values for position control
          .p(Constants.ElevatorConstants.ELEVATOR_P_VALUE)
          .d(Constants.ElevatorConstants.ELEVATOR_D_VALUE)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(5000)
          .maxAcceleration(75000)
          .allowedClosedLoopError(0.5);

      rightElevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // PID Values for position control
          .p(Constants.ElevatorConstants.ELEVATOR_P_VALUE)
          .d(Constants.ElevatorConstants.ELEVATOR_D_VALUE)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(5000)
          .maxAcceleration(75000)
          .allowedClosedLoopError(0.5);
    }
  }
}
