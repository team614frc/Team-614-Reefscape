package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoint;

public class ArmSubsystem extends SubsystemBase {
  // Arm Motor
  private final SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
  private final SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder();

  private final ProfiledPIDController armPid =
      new ProfiledPIDController(
          ArmConstants.kP,
          ArmConstants.kI,
          ArmConstants.kD,
          new TrapezoidProfile.Constraints(
              (ArmConstants.ARM_MAX_VELOCITY).in(RadiansPerSecond),
              (ArmConstants.ARM_MAX_ACCELERATION).in(RadiansPerSecondPerSecond)));

  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  // Default Current Target
  private ArmSetpoint armSetpoint = ArmSetpoint.START;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor.configure(
        Configs.ArmConfig.ARM_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean atSetpoint() {
    return Math.abs(armEncoder.getPosition() - armSetpoint.value.in(Rotations))
        <= ArmConstants.ARM_TOLERANCE.in(Rotations);
  }

  private Angle getPosition() {
    return Rotations.of(armEncoder.getPosition());
  }

  /** Drive the arm motor to its respective setpoint using PID and feedforward control. */
  private void runPID() {
    double armFeedforwardVoltage =
        armFeedforward.calculate(
            armPid.getSetpoint().position - ArmConstants.ARM_FEEDFORWARD_OFFSET.in(Radians),
            armPid.getSetpoint().velocity);

    double armPidOutput =
        armPid.calculate(getPosition().in(Radians), armSetpoint.value.in(Radians));

    armMotor.setVoltage(armPidOutput + armFeedforwardVoltage);

    SmartDashboard.putNumber("Arm FF", armFeedforwardVoltage);
  }

  /** Set the arm to a predefined setpoint. */
  public Command setSetpoint(ArmSetpoint setpoint) {
    return this.runOnce(() -> armSetpoint = setpoint);
  }

  @Override
  public void periodic() {
    runPID();

    SmartDashboard.putNumber("Arm Target Position", armSetpoint.value.in(Rotations));
    SmartDashboard.putNumber("Arm Actual Position", armEncoder.getPosition());
  }
}
