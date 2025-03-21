// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
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

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(
        run(
            () -> {
              if (DriverStation.isAutonomous()) {
                autoPattern().applyTo(buffer); // Autonomous mode pattern
              } else {
                LEDPattern.rainbow(255, 255)
                    .scrollAtAbsoluteSpeed(
                        Constants.LEDConstants.SCROLL_SPEED, Constants.LEDConstants.SPACING)
                    .applyTo(buffer);
              }
            }));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
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
    LEDPattern pattern;
    if (DriverStation.isAutonomous()) {
      pattern = LEDPattern.solid(Constants.LEDConstants.AUTO_COLOR);
    } else {
      pattern = LEDPattern.solid(Constants.LEDConstants.TELEOP_COLOR);
    }
    return run(() -> pattern.applyTo(buffer));
  }

  public Command setIntakePattern() {
    LEDPattern m_scrollingRainbow =
        LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(
                Constants.LEDConstants.SCROLL_SPEED, Constants.LEDConstants.SPACING);
    return run(() -> m_scrollingRainbow.applyTo(buffer));
  }

  public Command setScoringPattern() {

    LEDPattern pattern =
        LEDPattern.solid(Constants.LEDConstants.ALIGNMENT_COLOR)
            .breathe(Constants.LEDConstants.BREATHE_TIME);
    return runPattern(pattern);
  }
}
