package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * utility class for creating controller rumble commands
 */
public final class Rumble {
  private Rumble() {} // prevent instantiation

  /**
   * creates a command that rumbles the controller until interrupted
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @return command that rumbles until interrupted
   */
  public static Command rumble(GenericHID controller, double intensity) {
    return Commands.startEnd(
        () -> controller.setRumble(RumbleType.kBothRumble, intensity),
        () -> controller.setRumble(RumbleType.kBothRumble, 0.0));
  }

  /**
   * creates a command that rumbles the controller for a specified duration
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @param seconds duration in seconds
   * @return command that rumbles for the specified duration
   */
  public static Command rumblePulse(GenericHID controller, double intensity, double seconds) {
    return rumble(controller, intensity).withTimeout(seconds);
  }

  /**
   * creates a command that rumbles in a repeating pattern
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @param onSeconds duration of each rumble pulse
   * @param offSeconds pause between pulses
   * @param repeats number of pulses
   * @return command that rumbles in a pattern
   */
  public static Command rumblePattern(
      GenericHID controller, double intensity, double onSeconds, double offSeconds, int repeats) {
    return rumble(controller, intensity)
        .withTimeout(onSeconds)
        .andThen(Commands.waitSeconds(offSeconds))
        .repeatedly()
        .withTimeout((onSeconds + offSeconds) * repeats - offSeconds);
  }
}
