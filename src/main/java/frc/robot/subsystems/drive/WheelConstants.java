package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

/** Wheel-specific tuning constants for the drive subsystem. */
public class WheelConstants {
  // Wheel radius
  public static final Distance wheelRadius =
      switch (Constants.getWheels()) {
        case SPIKE_GRIP -> Inches.of(1.84); // TODO: Set actual Spike radius
        case BILLET -> Inches.of(1.84); // TODO: Set actual Billet radius
      };

  // Max speed at 12V (depends on wheel grip/size)
  public static final LinearVelocity speedAt12Volts =
      switch (Constants.getWheels()) {
        case SPIKE_GRIP -> MetersPerSecond.of(6.20); // TODO: Tune
        case BILLET -> MetersPerSecond.of(5.85); // TODO: Tune
      };

  // Slip current (depends on wheel grip)
  public static final Current slipCurrent =
      switch (Constants.getWheels()) {
        case SPIKE_GRIP -> Amps.of(14); // TODO: Tune
        case BILLET -> Amps.of(120); // TODO: Tune
      };

  // Drive motor PID gains (tuned per wheel type)
  public static final Slot0Configs driveGains =
      switch (Constants.getWheels()) {
        case SPIKE_GRIP -> new Slot0Configs()
            .withKP(0.01)
            .withKI(0)
            .withKD(0)
            .withKS(0.13506)
            .withKV(0.56417); // TODO: Tune
        case BILLET -> new Slot0Configs()
            .withKP(0.01)
            .withKI(0)
            .withKD(0)
            .withKS(0.13506)
            .withKV(0.56417); // TODO: Tune
      };

  // Steer gains (probably same for all wheels, but can differ)
  public static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(2.49)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
}
