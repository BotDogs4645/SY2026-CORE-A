package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double MAX_TORQUE_CURRENT_AMPS = 40.0;

  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final CoastOut coastRequest = new CoastOut();
  private final DutyCycleOut dutyCycleWant = new DutyCycleOut(0.0);
  private final TorqueCurrentFOC torqueCurrentWant = new TorqueCurrentFOC(0.0);

  public FlywheelIOTalonFX() {
    talon = new TalonFX(ShooterConstants.flywheelMotorId);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temperature = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temperature);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temperature);
    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    double currentVelocityRotPerSec = Units.radiansToRotations(velocity.getValueAsDouble());
    double targetVelocityRotPerSec = Units.radiansToRotations(outputs.velocityRadsPerSec);
    boolean belowTarget = currentVelocityRotPerSec < targetVelocityRotPerSec;

    switch (outputs.mode) {
      case COAST -> talon.setControl(coastRequest);
      case DUTY_CYCLE_BANG_BANG -> {
        // full duty cycle when below target and zero when above
        talon.setControl(dutyCycleWant.withOutput(belowTarget ? 1.0 : 0.0));
      }
      case TORQUE_CURRENT_BANG_BANG -> {
        // max torque current when below target and zero when above
        talon.setControl(torqueCurrentWant.withOutput(belowTarget ? MAX_TORQUE_CURRENT_AMPS : 0.0));
      }
    }
  }
}
