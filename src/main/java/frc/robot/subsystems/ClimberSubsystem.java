package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex leftClimberMotor =
      new SparkFlex(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);

private final SparkFlex rightClimberMotor =
      new SparkFlex(Constants.ClimberConstants.RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);


  SparkMaxConfig config = new SparkMaxConfig();

  public ClimberSubsystem() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit((int) (Constants.ClimberConstants.MOTOR_CURRENT_LIMIT.in(Amp)));
    leftClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  public void set(double speed) {
    leftClimberMotor.set(speed);
  }

}
