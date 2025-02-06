import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffectorSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class EndEffectorTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  EndEffectorSubsystem endEffector;
  SparkFlexSim m_simMotor;

  public EndEffectorTest() {}

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    endEffector = new EndEffectorSubsystem(); // Inject mocked motor into subsystem

    m_simMotor =
        new SparkFlexSim(endEffector.getMotor(), DCMotor.getNeoVortex(1)); // Simulate motor
    m_simMotor.enable();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_simMotor.disable();
    endEffector.getMotor().close();
  }

  @Test // marks this method as a test
  void intakeTest() {

    // executing the command
    endEffector.intake().execute();
    // When the command is executed we are getting the speed at the subsystem
    assertEquals(Constants.EndEffectorConstants.INTAKE_SPEED, endEffector.getSpeed());
  }

  @Test
  void outtakeTest() {
    endEffector.outtake().execute();
    assertEquals(Constants.EndEffectorConstants.OUTTAKE_SPEED, endEffector.getSpeed());
  }

  @Test
  void stopTest() {
    endEffector.stop().execute();
    assertEquals(0.0, endEffector.getSpeed());
  }
}
