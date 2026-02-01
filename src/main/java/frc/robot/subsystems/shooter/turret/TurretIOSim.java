package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class TurretIOSim implements TurretIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final int SIM_SUBSTEPS = 20; // ~1kHz to match TalonFX PID rate

  private final DCMotorSim sim;
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(MOTOR, 0.001, ShooterConstants.turretGearRatio),
            MOTOR);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // sim is advanced in applyOutputs via substeps
    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.torqueCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE, COAST -> {
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
        sim.update(Constants.loopPeriodSecs);
      }
      case CLOSED_LOOP -> {
        // substep the PD controller to approximate TalonFX's ~1kHz position PID loop.
        // without this, the 50Hz robot loop makes the controller feel weak at low gains
        // and oscillate at high gains.
        double subDt = Constants.loopPeriodSecs / SIM_SUBSTEPS;
        for (int i = 0; i < SIM_SUBSTEPS; i++) {
          double positionError =
              Units.radiansToRotations(outputs.position - sim.getAngularPositionRad());
          double velocityError =
              Units.radiansToRotations(outputs.velocity - sim.getAngularVelocityRadPerSec());
          double torqueCurrent = outputs.kP * positionError + outputs.kD * velocityError;

          // V = I*R + oh muh guh/Kv to produce the commanded torque current
          double motorVelocityRadPerSec =
              sim.getAngularVelocityRadPerSec() * ShooterConstants.turretGearRatio;
          appliedVolts =
              torqueCurrent * MOTOR.rOhms + motorVelocityRadPerSec / MOTOR.KvRadPerSecPerVolt;
          appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

          sim.setInputVoltage(appliedVolts);
          sim.update(subDt);
        }
      }
    }
  }
}
