package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** singleton LED subsystem with pattern methods */
public class Leds extends SubsystemBase {
  private static Leds instance;

  // hardware IO
  private final LedIO io;

  // public state flags
  public boolean lowBatteryAlert = false;
  public boolean endgameAlert = false;
  public boolean gamePieceLoaded = false; // future use

  // pattern state
  private boolean estopped = false;

  // colors
  private static final Color RED = new Color(255, 0, 0);
  private static final Color ORANGE = new Color(255, 100, 0);
  private static final Color GOLD = new Color(255, 200, 0);
  private static final Color GREEN = new Color(0, 255, 0);
  private static final Color BLUE = new Color(0, 0, 255);
  private static final Color OFF = new Color(0, 0, 0);

  /** returns the singleton instance */
  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds(new LedIO.LedIOStub());
    }
    return instance;
  }

  /** creates singleton instance with specific IO implementation, to be called once at startup */
  public static void initialize(LedIO io) {
    if (instance == null) {
      instance = new Leds(io);
    }
  }

  private Leds(LedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // check e-stop state
    estopped = DriverStation.isEStopped();

    // pattern priority (highest to lowest)
    if (estopped) {
      solid(RED);
    } else if (lowBatteryAlert) {
      strobe(ORANGE, RED, LedConstants.STROBE_DURATION);
    } else if (endgameAlert) {
      strobe(RED, GOLD, LedConstants.STROBE_DURATION);
    } else if (gamePieceLoaded) {
      solid(GREEN);
    } else if (DriverStation.isAutonomousEnabled()) {
      wave(GOLD, BLUE, LedConstants.WAVE_CYCLE_LENGTH, LedConstants.WAVE_DURATION);
    } else if (DriverStation.isTeleopEnabled()) {
      solid(getAllianceColor());
    } else {
      wave(getAllianceColor(), OFF, LedConstants.WAVE_CYCLE_LENGTH, LedConstants.WAVE_DURATION * 2);
    }

    // push to hardware
    io.update();

    // log state for debugging without hardware
    Logger.recordOutput("Leds/lowBatteryAlert", lowBatteryAlert);
    Logger.recordOutput("Leds/endgameAlert", endgameAlert);
    Logger.recordOutput("Leds/gamePieceLoaded", gamePieceLoaded);
    Logger.recordOutput("Leds/estopped", estopped);
  }

  /** sets all LEDs to a solid color */
  public void solid(Color color) {
    io.setAll(color);
  }

  /** alternates between two colors at the specified rate */
  public void strobe(Color color1, Color color2, double durationSeconds) {
    double time = Timer.getFPGATimestamp();
    boolean useFirst = ((int) (time / durationSeconds)) % 2 == 0;
    io.setAll(useFirst ? color1 : color2);
  }

  /** fades between two colors smoothly */
  public void breath(Color color1, Color color2, double durationSeconds) {
    double time = Timer.getFPGATimestamp();
    double progress = (time % durationSeconds) / durationSeconds;
    // sine wave for smooth breathing (0 to 1 to 0)
    double blend = (Math.sin(progress * 2 * Math.PI - Math.PI / 2) + 1) / 2;
    Color blended = interpolateColor(color1, color2, blend);
    io.setAll(blended);
  }

  /** creates a moving wave pattern along the LED strip. */
  public void wave(Color color1, Color color2, int cycleLength, double durationSeconds) {
    double time = Timer.getFPGATimestamp();
    double waveProgress = (time % durationSeconds) / durationSeconds;

    for (int i = 0; i < io.getLength(); i++) {
      double position = (double) i / cycleLength;
      double waveValue = (Math.sin((position + waveProgress) * 2 * Math.PI) + 1) / 2;
      Color color = interpolateColor(color1, color2, waveValue);
      io.setLED(i, color);
    }
  }

  /** returns alliance color */
  private Color getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red ? RED : BLUE;
    }
    return BLUE; // default to blue
  }

  /** linearly interpolates between two colors */
  private Color interpolateColor(Color color1, Color color2, double blend) {
    double r = color1.red + (color2.red - color1.red) * blend;
    double g = color1.green + (color2.green - color1.green) * blend;
    double b = color1.blue + (color2.blue - color1.blue) * blend;
    return new Color(r, g, b);
  }
}
