import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
  public SparkFlexSim m_simMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    endEffector = new EndEffectorSubsystem(); // Inject mocked motor into subsystem

    m_simMotor =
        new SparkFlexSim(
            new SparkFlex(Constants.EndEffectorConstants.END_EFFECTOR_MOTOR, MotorType.kBrushless),
            new DCMotor(12.0, 2.22, 100, 120, 0, 1)); // Simulate motor
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    endEffector.set(0); // destroy our end effector object
  }

  @Test // marks this method as a test
  void intakeTest() {
    endEffector.intake().schedule(); // close the intake
    assertEquals(Constants.EndEffectorConstants.INTAKE_SPEED, m_simMotor.getVelocity(), DELTA);
  }

  @Test
  void outtakeTest() {
    endEffector.outtake().schedule();
    assertEquals(Constants.EndEffectorConstants.OUTTAKE_SPEED, m_simMotor.getVelocity(), DELTA);
  }

  @Test
  void stopTest() {
    endEffector.stop();
    assertEquals(0.0, m_simMotor.getVelocity(), DELTA);
  }
}
