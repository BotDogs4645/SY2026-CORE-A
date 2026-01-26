package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** utility class for creating controller rumble commands */
public final class Rumble {
  private Rumble() {} // prevent instantiation

  /**
   * creates a command that rumbles both motors until interrupted
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @return command that rumbles until interrupted
   */
  public static Command rumble(GenericHID controller, double intensity) {
    return rumble(controller, intensity, RumbleType.kBothRumble);
  }

  /**
   * creates a command that rumbles a specific motor until interrupted
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @param type which rumble motor(s) to use
   * @return command that rumbles until interrupted
   */
  public static Command rumble(GenericHID controller, double intensity, RumbleType type) {
    return Commands.startEnd(
        () -> controller.setRumble(type, intensity), () -> controller.setRumble(type, 0.0));
  }

  /**
   * creates a command that rumbles only the left motor until interrupted
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @return command that rumbles until interrupted
   */
  public static Command rumbleLeft(GenericHID controller, double intensity) {
    return rumble(controller, intensity, RumbleType.kLeftRumble);
  }

  /**
   * creates a command that rumbles only the right motor until interrupted
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @return command that rumbles until interrupted
   */
  public static Command rumbleRight(GenericHID controller, double intensity) {
    return rumble(controller, intensity, RumbleType.kRightRumble);
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
   * creates a command that rumbles left motor for a specified duration
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @param seconds duration in seconds
   * @return command that rumbles for the specified duration
   */
  public static Command rumblePulseLeft(GenericHID controller, double intensity, double seconds) {
    return rumbleLeft(controller, intensity).withTimeout(seconds);
  }

  /**
   * creates a command that rumbles right motor for a specified duration
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @param seconds duration in seconds
   * @return command that rumbles for the specified duration
   */
  public static Command rumblePulseRight(GenericHID controller, double intensity, double seconds) {
    return rumbleRight(controller, intensity).withTimeout(seconds);
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

  /**
   * creates a command that alternates between left and right rumble
   *
   * @param controller the controller to rumble
   * @param intensity rumble intensity from 0.0 to 1.0
   * @param pulseSeconds duration of each side's pulse
   * @param repeats number of left-right cycles
   * @return command that alternates rumble between sides
   */
  public static Command rumbleAlternating(
      GenericHID controller, double intensity, double pulseSeconds, int repeats) {
    return rumbleLeft(controller, intensity)
        .withTimeout(pulseSeconds)
        .andThen(rumbleRight(controller, intensity).withTimeout(pulseSeconds))
        .repeatedly()
        .withTimeout(pulseSeconds * 2 * repeats);
  }
}
