package frc.robot.subsystems.leds;

/**
 * placeholder constants for LED hardware configuration, someone needs to update this once hardware
 * is decided
 */
public final class LedConstants {
  private LedConstants() {} // prevent instantiation

  // hardware configuration - TODO: update when known
  public static final int LED_COUNT = 60;
  public static final int LED_PWM_PORT = 0; // for AddressableLED
  public static final String WLED_ADDRESS = "10.46.45.10"; // for WLED

  // pattern timing
  public static final double STROBE_DURATION = 0.1; // 5 Hz
  public static final double BREATH_DURATION = 1.0;
  public static final double WAVE_DURATION = 2.0;
  public static final int WAVE_CYCLE_LENGTH = 25;
}
