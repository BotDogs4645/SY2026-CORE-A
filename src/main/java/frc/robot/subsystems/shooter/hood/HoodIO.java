package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  enum HoodIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  class HoodIOOutputs {
    public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void applyOutputs(HoodIOOutputs outputs) {}
}
