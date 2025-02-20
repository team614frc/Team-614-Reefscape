import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTest {
  private static final double DELTA = 1e-2;
  private IntakeSubsystem intake;
  private SparkFlexSim simMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    intake = new IntakeSubsystem();

    simMotor = new SparkFlexSim(intake.getMotor(), DCMotor.getNeoVortex(1));
    simMotor.enable();
  }

  @AfterEach
  void shutdown() throws Exception {
    simMotor.disable();
    intake.getMotor().close();
  }

  @Test
  void intakeGamepieceTest() {
    Command command = intake.intakeGamepiece();

    command.initialize();
    assertEquals(Constants.IntakeConstants.INTAKE_REST_SPEED, intake.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.IntakeConstants.INTAKE_SPEED, intake.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.IntakeConstants.INTAKE_REST_SPEED, intake.getMotor().get(), DELTA);
  }

  @Test
  void outtakeGamepieceTest() {
    Command command = intake.outtakeGamepiece();

    command.initialize();
    assertEquals(Constants.IntakeConstants.OUTTAKE_SPEED, intake.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.IntakeConstants.OUTTAKE_SPEED, intake.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.IntakeConstants.OUTTAKE_REST_SPEED, intake.getMotor().get(), DELTA);
  }

  @Test
  void stopIntakeTest() {
    Command command = intake.stopIntake();

    command.initialize();
    assertEquals(Constants.IntakeConstants.INTAKE_REST_SPEED, intake.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.IntakeConstants.INTAKE_REST_SPEED, intake.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.IntakeConstants.INTAKE_REST_SPEED, intake.getMotor().get(), DELTA);
  }
}
