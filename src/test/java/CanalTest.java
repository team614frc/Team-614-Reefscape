import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.CanalSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CanalTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  private CanalSubsystem canal;
  private SparkFlexSim simMotor;

  public CanalTest() {}

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

  @Test // marks this method as a test
  void intakeTest() {

    // executing the command
    canal.intake().execute();
    // When the command is executed we are getting the speed at the subsystem
    assertEquals(Constants.CanalConstants.INTAKE_SPEED, canal.getMotor().get());
  }

  @Test
  void outtakeTest() {
    canal.outtake().execute();
    assertEquals(Constants.CanalConstants.OUTTAKE_SPEED, canal.getMotor().get());
  }
}
