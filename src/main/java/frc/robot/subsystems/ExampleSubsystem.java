package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ExampleSubsystem extends SubsystemBase {
  private final SparkFlex flexMotor = new SparkFlex(16, MotorType.kBrushless);

  private SparkAnalogSensor flexEncoder = flexMotor.getAnalog();

  public ExampleSubsystem() {
    flexMotor.configure(
        Configs.ExampleSubsystem.EXAMPLE_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("External Encoder Position", flexEncoder.getPosition());
  }
}
