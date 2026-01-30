package frc.robot.subsystems.shooter.turret;

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
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final NeutralOut neutralRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0);

  private double lastKP = Double.NaN;
  private double lastKD = Double.NaN;

  public TurretIOTalonFX() {
    talon = new TalonFX(ShooterConstants.turretMotorId, DriveConstants.canBus);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.turretGearRatio;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
    inputs.motorConnected = connectedDebounce.calculate(status.isOK());
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> talon.setControl(neutralRequest);
      case COAST -> talon.setControl(coastRequest);
      case CLOSED_LOOP -> {
        if (outputs.kP != lastKP || outputs.kD != lastKD) {
          lastKP = outputs.kP;
          lastKD = outputs.kD;
          var slot0 = new Slot0Configs();
          slot0.kP = outputs.kP;
          slot0.kD = outputs.kD;
          talon.getConfigurator().apply(slot0);
        }

        double positionRotations = Units.radiansToRotations(outputs.position);
        double velocityRotPerSec = Units.radiansToRotations(outputs.velocity);
        talon.setControl(
            positionRequest
                .withPosition(positionRotations)
                .withVelocity(velocityRotPerSec)
                .withSlot(0));
      }
    }
  }
}
