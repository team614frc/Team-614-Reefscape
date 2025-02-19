import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.ClimberSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

class ClimbTest {
  public static final double DELTA = 1e-2; // acceptable deviation range
  private ClimberSubsystem climber;
  private SparkFlexSim simMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    climber = new ClimberSubsystem(); // Inject mocked motor into subsystem

    simMotor = new SparkFlexSim(climber.getMotor(), DCMotor.getNeoVortex(1)); // Simulate motor
    simMotor.enable();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    simMotor.disable();
    climber.getMotor().close();
  }
}
