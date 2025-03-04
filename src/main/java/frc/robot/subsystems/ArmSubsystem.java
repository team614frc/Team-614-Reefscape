package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoint;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {
  // Arm Motor
  private SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
  private SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder();

  private final ProfiledPIDController armPid =
      new ProfiledPIDController(
          ArmConstants.kP,
          ArmConstants.kI,
          ArmConstants.kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(ArmConstants.ARM_MAX_VELOCITY),
              Units.rotationsToRadians(ArmConstants.ARM_MAX_ACCELERATION)));

  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  // Default Current Target
  private double armSetpoint = ArmSetpoint.armStart.value;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor.configure(
        Configs.ArmConfig.ARM_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean atSetpoint() {
    return Math.abs(armEncoder.getPosition() - armSetpoint) <= ArmConstants.ARM_TOLERANCE;
  }

  private double getArmAngleRadians() {
    return Units.rotationsToRadians(armEncoder.getPosition());
  }

  public boolean checkArmL3() {
    boolean a = armSetpoint == ArmSetpoint.armL3.value;
    boolean b =
        Math.abs(armEncoder.getPosition() - ArmSetpoint.armL3.value) <= ArmConstants.ARM_TOLERANCE;

    return (a || b);
  }

  public boolean checkArmPuke() {
    boolean a = armSetpoint == ArmSetpoint.pukeArm.value;
    boolean b =
        Math.abs(armEncoder.getPosition() - ArmSetpoint.pukeArm.value)
            <= ElevatorConstants.ELEVATOR_TOLERANCE;

    return a || b;
  }

  public boolean checkArmHover() {
    boolean a = armSetpoint == ArmSetpoint.armHover.value;
    boolean b =
        Math.abs(armEncoder.getPosition() - ArmSetpoint.armHover.value)
            <= ArmConstants.ARM_TOLERANCE;

    return (a || b);
  }

  /** Drive the arm motor to its respective setpoint using PID and feedforward control. */
  private void runPID() {
    double armFeedforwardVoltage =
        armFeedforward.calculate(
            armPid.getSetpoint().position
                - Units.rotationsToRadians(ArmConstants.ARM_FEEDFORWARD_OFFSET),
            armPid.getSetpoint().velocity);

    double armPidOutput =
        armPid.calculate(getArmAngleRadians(), Units.rotationsToRadians(armSetpoint));

    armMotor.setVoltage(armPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Arm FF", armFeedforwardVoltage);
  }

  /** Set the arm to a predefined setpoint. */
  public Command setSetpoint(ArmSetpoint setpoint) {
    return this.runOnce(() -> armSetpoint = setpoint.value);
  }

  @Override
  public void periodic() {
    runPID();

    SmartDashboard.putNumber("Arm Target Position", armSetpoint);
    SmartDashboard.putNumber("Arm Actual Position", armEncoder.getPosition());
  }
}
