package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  enum FlywheelIOOutputMode {
    COAST,
    DUTY_CYCLE_BANG_BANG, // BANNNNNGGGG GREEEENNN FN
    TORQUE_CURRENT_BANG_BANG
  }

  class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
