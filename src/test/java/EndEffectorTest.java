import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffectorSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class EndEffectorTest {
  private static final double DELTA = 1e-2;
  private EndEffectorSubsystem endEffector;
  private SparkFlexSim simMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    endEffector = new EndEffectorSubsystem();

    simMotor = new SparkFlexSim(endEffector.getMotor(), DCMotor.getNeoVortex(1));
    simMotor.enable();
  }

  @AfterEach
  void shutdown() throws Exception {
    simMotor.disable();
    endEffector.getMotor().close();
  }

  @Test
  void intakeTest() {
    Command command = endEffector.intake();

    command.initialize();
    assertEquals(Constants.EndEffectorConstants.INTAKE_SPEED, endEffector.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.EndEffectorConstants.INTAKE_SPEED, endEffector.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.EndEffectorConstants.INTAKE_SPEED, endEffector.getMotor().get(), DELTA);
  }

  @Test
  void outtakeTest() {
    Command command = endEffector.outtake();

    command.initialize();
    assertEquals(Constants.EndEffectorConstants.OUTTAKE_SPEED, endEffector.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.EndEffectorConstants.OUTTAKE_SPEED, endEffector.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.EndEffectorConstants.OUTTAKE_SPEED, endEffector.getMotor().get(), DELTA);
  }

  @Test
  void stopTest() {
    Command command = endEffector.stop();

    command.initialize();
    assertEquals(
        Constants.EndEffectorConstants.INTAKE_REST_SPEED, endEffector.getMotor().get(), DELTA);

    command.execute();
    assertEquals(
        Constants.EndEffectorConstants.INTAKE_REST_SPEED, endEffector.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(
        Constants.EndEffectorConstants.INTAKE_REST_SPEED, endEffector.getMotor().get(), DELTA);
  }
}
