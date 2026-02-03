package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }

  enum TurretIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  class TurretIOOutputs {
    public TurretIOOutputMode mode = TurretIOOutputMode.BRAKE;
    public double position = 0.0;
    public double velocity = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void applyOutputs(TurretIOOutputs outputs) {}
}
