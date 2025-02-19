import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.CanalSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

class CanalTest {
  private CanalSubsystem canal;
  private SparkFlexSim simMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    canal = new CanalSubsystem(); // Inject mocked motor into subsystem

    simMotor = new SparkFlexSim(canal.getMotor(), DCMotor.getNeoVortex(1)); // Simulate motor
    simMotor.enable();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    simMotor.disable();
    canal.getMotor().close();
  }
}
