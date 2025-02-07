import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  IntakeSubsystem intake;
  SparkFlexSim simMotor;

  public IntakeTest() {}

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    intake = new IntakeSubsystem(); // Inject mocked motor into subsystem

    simMotor = new SparkFlexSim(intake.getMotor(), DCMotor.getNeoVortex(1)); // Simulate motor
    simMotor.enable();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    simMotor.disable();
    intake.getMotor().close();
  }

  @Test // marks this method as a test
  void intakeGamepieceTest() {
    // executing the command
    intake.intakeGamepiece().execute();
    // When the command is executed we are getting the speed at the subsystem
    assertEquals(Constants.IntakeConstants.INTAKE_SPEED, intake.getSpeed());
  }

  @Test
  void pukeGamepieceTest() {
    intake.pukeGamepiece().execute();
    assertEquals(Constants.IntakeConstants.OUTTAKE_SPEED, intake.getSpeed());
  }

  @Test
  void stopIntakeTest() {
    intake.stopIntake().execute();
    assertEquals(0.0, intake.getSpeed());
  }
}
