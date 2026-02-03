package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim sim;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(MOTOR, 0.025, ShooterConstants.flywheelGearRatio),
            MOTOR);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.torqueCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.mode == FlywheelIOOutputMode.COAST) {
      appliedVolts = 0.0;
    } else if (outputs.mode == FlywheelIOOutputMode.CLOSED_LOOP) {
      double currentRotPerSec = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
      double targetRotPerSec = Units.radiansToRotations(outputs.velocityRadsPerSec);

      double ffVolts = targetRotPerSec * outputs.kV;

      double errorRotPerSec = targetRotPerSec - currentRotPerSec;
      double pidVolts = errorRotPerSec * outputs.kP;

      appliedVolts = ffVolts + pidVolts;
      appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void simulateShotDisturbance() {

    sim.setState(sim.getAngularPositionRad(), sim.getAngularVelocityRadPerSec() - 30.0);
  }
}
