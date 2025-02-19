import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.EndEffectorSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

class EndEffectorTest {
  private EndEffectorSubsystem endEffector;
  private SparkFlexSim simMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    endEffector = new EndEffectorSubsystem(); // Inject mocked motor into subsystem

    simMotor = new SparkFlexSim(endEffector.getMotor(), DCMotor.getNeoVortex(1)); // Simulate motor
    simMotor.enable();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    simMotor.disable();
    endEffector.getMotor().close();
  }
}
