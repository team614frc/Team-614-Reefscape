package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSetpoint;

public class ElevatorSubsystem extends SubsystemBase {

  // Elevator Motor
  private final SparkFlex elevatorMotor =
      new SparkFlex(ElevatorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder elevatorEncoder = elevatorMotor.getExternalEncoder();

  private final ProfiledPIDController elevatorPid =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.ELEVATOR_MAX_VELOCITY,
              ElevatorConstants.ELEVATOR_MAX_ACCELERATION));

  // Default Current Target
  private ElevatorSetpoint elevatorSetpoint = ElevatorSetpoint.IDLE;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor.configure(
        Configs.ElevatorConfig.ELEVATOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    elevatorEncoder.setPosition(0);
  }

  public boolean atSetpoint(ElevatorSetpoint setpoint) {
    boolean a = elevatorSetpoint == setpoint;
    boolean b =
        Math.abs(elevatorEncoder.getPosition() - setpoint.value.in(Rotations))
            <= ElevatorConstants.ELEVATOR_TOLERANCE.in(Rotations);
    return a || b;
  }

  private double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command descendSlowly() {
    return Commands.runEnd(
        () -> elevatorMotor.set(ElevatorConstants.ELEVATOR_SLOW_DOWN_SPEED),
        () -> elevatorMotor.set(ElevatorConstants.ELEVATOR_STOP_SPEED),
        this);
  }

  public boolean isStalled() {
    return Math.abs(elevatorEncoder.getVelocity()) < ElevatorConstants.ELEVATOR_STALL_VELOCITY
        && elevatorMotor.get() == ElevatorConstants.ELEVATOR_SLOW_DOWN_SPEED;
  }

  public Command resetEncoder() {
    return Commands.runOnce(() -> elevatorEncoder.setPosition(0));
  }

  /** Drive the elevator motor to its respective setpoint using PID control. */
  private void runPID() {
    double elevatorPidOutput =
        elevatorPid.calculate(getPosition(), elevatorSetpoint.value.in(Rotations));
    elevatorMotor.setVoltage(elevatorPidOutput);
  }

  /** Set the elevator to a predefined setpoint. */
  public Command setSetpoint(ElevatorSetpoint setpoint) {
    return this.runOnce(() -> elevatorSetpoint = setpoint);
  }

  @Override
  public void periodic() {
    runPID();

    SmartDashboard.putNumber("Elevator Target Position", elevatorSetpoint.value.in(Rotations));
    SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator Stalled", isStalled());
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());
  }
}
