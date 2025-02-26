import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ClimbTest {
  public static final double DELTA = 1e-2;
  private ClimberSubsystem climber;
  private SparkFlexSim simMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    climber = new ClimberSubsystem();

    simMotor = new SparkFlexSim(climber.getMotor(), DCMotor.getNeoVortex(1));
    simMotor.enable();
  }

  @AfterEach
  void shutdown() throws Exception {
    simMotor.disable();
    climber.getMotor().close();
  }

  @Test
  void climbTest() {
    Command command = climber.climb();

    command.initialize();
    assertEquals(Constants.ClimberConstants.CLIMB_REST_SPEED, climber.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.ClimberConstants.CLIMB_SPEED, climber.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.ClimberConstants.CLIMB_REST_SPEED, climber.getMotor().get(), DELTA);
  }

  @Test
  void reverseClimbTest() {
    Command command = climber.reverseClimb();

    command.initialize();
    assertEquals(Constants.ClimberConstants.CLIMB_REST_SPEED, climber.getMotor().get(), DELTA);

    command.execute();
    assertEquals(Constants.ClimberConstants.REVERSE_CLIMB_SPEED, climber.getMotor().get(), DELTA);

    command.end(false);
    assertEquals(Constants.ClimberConstants.CLIMB_REST_SPEED, climber.getMotor().get(), DELTA);
  }
}
