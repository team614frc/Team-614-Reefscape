import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CanalSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CanalTest {
  private static final double DELTA = 1e-2;
  private CanalSubsystem canal;
  private SparkFlexSim simMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    canal = new CanalSubsystem();

    simMotor = new SparkFlexSim(canal.getMotor(), DCMotor.getNeoVortex(1));
    simMotor.enable();
  }

  @AfterEach
  void shutdown() throws Exception {
    simMotor.disable();
    canal.getMotor().close();
  }

  @Test
  void intakeTest() {
    Command command = canal.intake();

    command.initialize();
    assertEquals(Constants.CanalConstants.INTAKE_SPEED, canal.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.CanalConstants.INTAKE_SPEED, canal.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.CanalConstants.INTAKE_SPEED, canal.getMotor().get(), DELTA);
  }

  @Test
  void outtakeTest() {
    Command command = canal.outtake();

    command.initialize();
    assertEquals(Constants.CanalConstants.OUTTAKE_SPEED, canal.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.CanalConstants.OUTTAKE_SPEED, canal.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.CanalConstants.OUTTAKE_REST_SPEED, canal.getMotor().get(), DELTA);
  }
}
