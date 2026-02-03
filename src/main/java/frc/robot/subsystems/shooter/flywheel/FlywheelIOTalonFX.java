package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final CoastOut coastRequest = new CoastOut();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private double lastkV = -1;
  private double lastkP = -1;

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
    if (outputs.mode == FlywheelIOOutputMode.COAST) {
      talon.setControl(coastRequest);
    } else {
      if (outputs.kV != lastkV || outputs.kP != lastkP) {
        var slot0 = new Slot0Configs();
        slot0.kP = outputs.kP;
        slot0.kV = outputs.kV;
        talon.getConfigurator().apply(slot0);
        lastkV = outputs.kV;
        lastkP = outputs.kP;
      }

      double targetRotPerSec = Units.radiansToRotations(outputs.velocityRadsPerSec);

      talon.setControl(
          velocityRequest.withVelocity(targetRotPerSec).withSlot(0).withEnableFOC(true));
    }
  }
}
