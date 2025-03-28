// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 120;
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDSubsystem() {
    led = new AddressableLED(kPort);
    buffer = new AddressableLEDBuffer(kLength);
    led.setLength(kLength);
    led.start();
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    if (!DriverStation.isDSAttached()) {
      LEDPattern blinkPattern =
          LEDPattern.solid(Constants.LEDConstants.TELEOP_COLOR).blink(Seconds.of(1));
      blinkPattern.applyTo(buffer);
    } else if (DriverStation.isAutonomousEnabled()) {
      LEDPattern m_scrollingRainbow =
          LEDPattern.rainbow(255, 255)
              .scrollAtAbsoluteSpeed(
                  Constants.LEDConstants.SCROLL_SPEED, Constants.LEDConstants.SPACING);
      m_scrollingRainbow.applyTo(buffer);
    } else if (DriverStation.isTeleopEnabled()) {
      return;
    } else if (DriverStation.isDSAttached()) {
      LEDPattern ready = LEDPattern.solid(Constants.LEDConstants.READY_COLOR);
      ready.applyTo(buffer);
    }
    led.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }

  public static LEDPattern autoPattern() {
    return LEDPattern.solid(Constants.LEDConstants.AUTO_COLOR);
  }

  public static LEDPattern teleopPattern() {
    return LEDPattern.solid(Constants.LEDConstants.TELEOP_COLOR);
  }

  public Command setBasicPattern() {
    LEDPattern m_scrollingRainbow =
    LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(
            Constants.LEDConstants.SCROLL_SPEED, Constants.LEDConstants.SPACING);

    return run(
        () -> {
          m_scrollingRainbow.applyTo(buffer);
          led.setData(buffer);
        });
  }

  public Command canalPattern() {
    LEDPattern base =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlue, Color.kAqua);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Seconds).of(85));

    return run(
        () -> {
          pattern.applyTo(buffer);
          led.setData(buffer);
        });
  }

  public Command hasGamePiece() {
    LEDPattern blinkPattern =
    LEDPattern.solid(Constants.LEDConstants.TELEOP_COLOR).blink(Seconds.of(0.2));
    
  return run(
    () -> {
      blinkPattern.applyTo(buffer);
      led.setData(buffer);
    });
  }
}
