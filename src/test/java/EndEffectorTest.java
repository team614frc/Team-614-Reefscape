import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffectorSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class EndEffectorTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  private EndEffectorSubsystem endEffector;
  private CANSparkFlex mockMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    mockMotor = mock(SparkFlex.class); // Create a mocked SparkFlex motor
    endEffector = new EndEffectorSubsystem(mockMotor); // Inject mocked motor into subsystem
    motorSim = new PWMSim(Constants.EndEffectorConstants.END_EFFECTOR_MOTOR); // Simulate motor
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    endEffector.close(); // destroy our End Effector object
  }

  @Test // marks this method as a test
  void intake() {
    endEffector.intakeTest(); // close the intake
    assertEquals(EndEffectorConstants.INTAKE_SPEED, simMotor.getSpeed(), DELTA); // make sure that the value set to the motor is 0
  }

  @Test
  void outtake() {
    endEffector.outtakeTest();
    assertEquals(EndEffectorConstants.OUTTAKE_SPEED, simMotor.getSpeed(), DELTA);
  }

  @Test
  void stopTest() {
    endEffector.stop();
    assertEquals(0.0, motorSim.getSpeed(), DELTA);
  }
}