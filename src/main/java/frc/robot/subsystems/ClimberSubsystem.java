package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex climberMotor =
      new SparkFlex(Constants.ClimberConstants.CLIMBER_MOTOR, MotorType.kBrushless);

  public ClimberSubsystem() {
    climberMotor.configure(
        Configs.ClimberSubsystem.CLIMBER_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public SparkFlex getMotor() {
    return climberMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  public void set(double speed) {
    climberMotor.set(speed);
  }

  public Command climb() {
    return Commands.runEnd(
        () -> set(ClimberConstants.CLIMB_SPEED), () -> set(ClimberConstants.CLIMB_REST_SPEED));
  }

  public Command reverseClimb() {
    return Commands.runEnd(
        () -> set(ClimberConstants.REVERSE_CLIMB_SPEED),
        () -> set(ClimberConstants.CLIMB_REST_SPEED));
  }
}
