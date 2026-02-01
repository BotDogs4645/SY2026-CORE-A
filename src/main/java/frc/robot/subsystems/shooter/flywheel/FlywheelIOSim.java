package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
    switch (outputs.mode) {
      case COAST -> {
        appliedVolts = 0.0;
      }
      case DUTY_CYCLE_BANG_BANG -> {
        // full voltage when below target and zero when above
        if (sim.getAngularVelocityRadPerSec() < outputs.velocityRadsPerSec) {
          appliedVolts = 12.0;
        } else {
          appliedVolts = 0.0;
        }
      }
      case TORQUE_CURRENT_BANG_BANG -> {
        // full voltage when below target and zero when above (as close as the sim can get to torque
        // current)
        if (sim.getAngularVelocityRadPerSec() < outputs.velocityRadsPerSec) {
          appliedVolts = 12.0;
        } else {
          appliedVolts = 0.0;
        }
      }
    }

    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void simulateShotDisturbance() {
    // simulate energy transfer to a ball exiting the flywheel
    // drop velocity enough to trip the shot detection threshold (>10 rad/s)
    sim.setState(sim.getAngularPositionRad(), sim.getAngularVelocityRadPerSec() - 30.0);
  }
}
