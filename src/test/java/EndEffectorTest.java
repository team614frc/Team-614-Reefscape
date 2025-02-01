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
  private EndEffectorSubsystem endEffector;
  private SparkFlexSim m_simMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    endEffector = new EndEffectorSubsystem(); // create end
    m_simMotor =
        new SparkFlexSim(endEffector.getMotor(), DCMotor.getNeoVortex(1)); // Simulate motor
    // Simulate motor
    m_simMotor.enable();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    endEffector.getMotor().close(); // destroy our end effector object
  }

  @Test // marks this method as a test
  void intakeTest() {
    endEffector.intake(); // close the intake
    m_simMotor.setAppliedOutput(Constants.EndEffectorConstants.INTAKE_SPEED);
    assertEquals(Constants.EndEffectorConstants.INTAKE_SPEED, m_simMotor.getAppliedOutput(), DELTA);
  }

  @Test
  void outtakeTest() {
    endEffector.outtake();
    m_simMotor.setAppliedOutput(Constants.EndEffectorConstants.OUTTAKE_SPEED);
    assertEquals(
        Constants.EndEffectorConstants.OUTTAKE_SPEED, m_simMotor.getAppliedOutput(), DELTA);
  }

  @Test
  void stopTest() {
    endEffector.stop();
    assertEquals(0.0, m_simMotor.getVelocity(), DELTA);
  }
}
