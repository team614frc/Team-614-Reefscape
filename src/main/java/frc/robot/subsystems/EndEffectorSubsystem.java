package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    SmartDashboard.putNumber("End Effector Current", endEffectorCurrent());
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
    return this.runOnce(() -> set(EndEffectorConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return this.runOnce(() -> set(EndEffectorConstants.OUTTAKE_SPEED));
  }

  public Command punchAlgae() {
    return this.runOnce(
        () -> {
          set(EndEffectorConstants.OUTTAKE_ALGAE);
        });
  }

  // Command to deactivate the end effector (e.g., stop gripping or releasing)
  public Command stop() {
    return this.runOnce(
        () -> {
          set(EndEffectorConstants.INTAKE_REST_SPEED); // Stop the motor
        });
  }

  public boolean hasGamePiece() {
    double current = endEffectorMotor.getOutputCurrent();
    double output = endEffectorMotor.get();

    return (output > EndEffectorConstants.END_EFFECTOR_MIN_OUTPUT)
        && (current > EndEffectorConstants.MIN_CURRENT);
  }

  private double getEndEffectorCurrent() {
    return endEffectorMotor.getOutputCurrent();
  }
}
