package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkFlex endEffectorMotor =
      new SparkFlex(Constants.EndEffectorConstants.END_EFFECTOR_MOTOR, MotorType.kBrushless);

  public EndEffectorSubsystem() {
    endEffectorMotor.configure(
        Configs.EndEffectorConfig.END_EFFECTOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    SmartDashboard.putNumber("End Effector Motor Output", endEffectorMotor.get());
    SmartDashboard.putBoolean("End Effector Has Game Piece", hasGamePiece());
  }

  public SparkFlex getMotor() {
    return endEffectorMotor;
  }

  // Set the power level for the end effector motor
  public void set(double speed) {
    endEffectorMotor.set(speed);
  }

  // Command to activate the end effector (e.g., for gripping or releasing)
  public Command intake() {
    return Commands.runOnce(() -> set(EndEffectorConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return Commands.runOnce(() -> set(EndEffectorConstants.OUTTAKE_SPEED));
  }

  // Command to deactivate the end effector (e.g., stop gripping or releasing)
  public Command stop() {
    return Commands.runOnce(
        () -> {
          set(EndEffectorConstants.INTAKE_REST_SPEED); // Stop the motor
        });
  }

  public Command stall() {
    return Commands.runOnce(
        () -> {
          set(EndEffectorConstants.STALL_SPEED);
        });
  }

  public boolean hasGamePiece() {
    double velocity = endEffectorMotor.getEncoder().getVelocity();
    double output = endEffectorMotor.get();

    return (output > EndEffectorConstants.END_EFFECTOR_MIN_OUTPUT)
        && (velocity
            < EndEffectorConstants.END_EFFECTOR_MIN_RPM.in(
                Rotations)); // velocty ensures the motor is running before detecting a
    // stall
  }
}
