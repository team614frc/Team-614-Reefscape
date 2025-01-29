package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex ClimberMotor =
      new SparkFlex(Constants.ClimberConstants.CLIMBER_MOTOR, MotorType.kBrushless);

  public ClimberSubsystem() {
    ClimberMotor.configure(
        Configs.ClimberSubsystem.CLIMBER_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  public void set(double speed) {
    ClimberMotor.set(speed);
  }
}
