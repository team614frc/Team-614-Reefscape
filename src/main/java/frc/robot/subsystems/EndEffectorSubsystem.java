package frc.robot.subsystems;

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
        Configs.EndEffectorSubsystem.END_EFFECTOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    SmartDashboard.putNumber("End Effector Motor Output", endEffectorMotor.get());
    SmartDashboard.putBoolean("End Effector Has Game Piece", hasGamePiece());
  }

  // Set the power level for the end effector motor
  public void set(double speed) {
    endEffectorMotor.set(speed);
  }

  // Command to activate the end effector (e.g., for gripping or releasing)
  public Command intake() {
    return Commands.runOnce(
        () -> {
          set(Constants.EndEffectorConstants.INTAKE_SPEED); // Example speed value
        });
  }

  public Command outtake() {
    return Commands.runEnd(
        () -> set(Constants.EndEffectorConstants.OUTTAKE_SPEED), // Outtake speed (negative value)
        () -> set(0) // Stop motor when command ends
        );
  }

  // Command to deactivate the end effector (e.g., stop gripping or releasing)
  public Command stopEndEffector() {
    return Commands.runOnce(
        () -> {
          set(0); // Stop the motor
        });
  }

  public boolean hasGamePiece() {
    double velocity = endEffectorMotor.getEncoder().getVelocity();
    double output = endEffectorMotor.get();

    return (output > EndEffectorConstants.END_EFFECTOR_MINOUTPUT)
        && (velocity
            < EndEffectorConstants
                .END_EFFECTOR_MINRPM); // velocty ensures the motor is running before detecting a
    // stall
  }

  private boolean previouslyHadGamePiece = false;

  public boolean justPickedUpGamePiece() {
    boolean currentHasGamePiece = hasGamePiece();
    boolean justPickedUp = !previouslyHadGamePiece && currentHasGamePiece;
    previouslyHadGamePiece = currentHasGamePiece; // Update state for next check
    return justPickedUp;
  }
}
