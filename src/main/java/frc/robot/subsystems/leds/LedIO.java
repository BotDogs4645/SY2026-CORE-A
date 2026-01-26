package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

/** TODO: hardware abstraction interface for LED control */
public interface LedIO {
  /** set a single LED by index */
  void setLED(int index, Color color);

  /** set all LEDs to same color */
  void setAll(Color color);

  /** set a range of LEDs */
  void setRange(int start, int end, Color color);

  /** push buffer to hardware */
  void update();

  /** get total LED count */
  int getLength();

  /** default no-op implementation for when LEDs not configured */
  class LedIOStub implements LedIO {
    @Override
    public void setLED(int index, Color color) {}

    @Override
    public void setAll(Color color) {}

    @Override
    public void setRange(int start, int end, Color color) {}

    @Override
    public void update() {}

    @Override
    public int getLength() {
      return 0;
    }
  }
}
