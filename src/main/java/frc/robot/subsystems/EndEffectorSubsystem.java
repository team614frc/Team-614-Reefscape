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

public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkFlex endEffectorMotor =
      new SparkFlex(Constants.EndEffectorConstants.END_EFFECTOR_MOTOR, MotorType.kBrushless);

  public EndEffectorSubsystem() {
    endEffectorMotor.configure(
        Configs.EndEffectorSubsystem.endEffectorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  // Set the power level for the end effector motor
  public void set(double speed) {
    endEffectorMotor.set(speed);
  }

  // Command to activate the end effector (e.g., for gripping or releasing)
  public Command intake() {
    return Commands.runEnd(
        () -> {
          set(Constants.EndEffectorConstants.INTAKE_SPEED); // Example speed value
        },
        () -> {
          set(0); // Stop the motor when the command ends
        });
  }

  public Command outtake() {
    return Commands.runEnd(
        () -> set(Constants.EndEffectorConstants.OUTTAKE_SPEED), // Outtake speed (negative value)
        () -> set(0) // Stop motor when command ends
        );
  }

  // Command to deactivate the end effector (e.g., stop gripping or releasing)
  public Command stop() {
    return Commands.runEnd(
        () -> {
          set(0); // Stop the motor
        },
        () -> {
          set(0); // Ensure motor stays stopped after command ends
        });
  }
}
