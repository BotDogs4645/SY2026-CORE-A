package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final NeutralOut neutralRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0);

  private double lastKP = Double.NaN;
  private double lastKD = Double.NaN;

  public HoodIOTalonFX() {
    talon = new TalonFX(ShooterConstants.hoodMotorId);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.hoodGearRatio;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

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
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temperature);
    inputs.motorConnected = connectedDebounce.calculate(status.isOK());
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> talon.setControl(neutralRequest);
      case COAST -> talon.setControl(coastRequest);
      case CLOSED_LOOP -> {
        // only if changed
        if (outputs.kP != lastKP || outputs.kD != lastKD) {
          lastKP = outputs.kP;
          lastKD = outputs.kD;
          var slot0 = new Slot0Configs();
          slot0.kP = outputs.kP;
          slot0.kD = outputs.kD;
          talon.getConfigurator().apply(slot0);
        }

        double positionRotations = Units.radiansToRotations(outputs.positionRad);
        double velocityRotPerSec = Units.radiansToRotations(outputs.velocityRadsPerSec);
        talon.setControl(
            positionRequest
                .withPosition(positionRotations)
                .withVelocity(velocityRotPerSec)
                .withSlot(0));
      }
    }
  }
}
