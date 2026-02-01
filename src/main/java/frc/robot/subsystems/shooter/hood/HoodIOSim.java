package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class HoodIOSim implements HoodIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final int SIM_SUBSTEPS = 20; // ~1kHz to match TalonFX PID rate

  private final SingleJointedArmSim sim;
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    sim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                MOTOR, 0.01, ShooterConstants.hoodGearRatio),
            MOTOR,
            ShooterConstants.hoodGearRatio,
            0.3, // TODO: arm length meters
            Hood.minAngleRad,
            Hood.maxAngleRad,
            true, // simulate gravity
            Hood.minAngleRad);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // sim is advanced in applyOutputs via substeps
    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.torqueCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE, COAST -> {
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
        sim.update(Constants.loopPeriodSecs);
      }
      case CLOSED_LOOP -> {
        // substep the PD controller to approximate TalonFX's ~1kHz position PID loop
        double subDt = Constants.loopPeriodSecs / SIM_SUBSTEPS;
        for (int i = 0; i < SIM_SUBSTEPS; i++) {
          double positionError = Units.radiansToRotations(outputs.positionRad - sim.getAngleRads());
          double velocityError =
              Units.radiansToRotations(outputs.velocityRadsPerSec - sim.getVelocityRadPerSec());
          double torqueCurrent = outputs.kP * positionError + outputs.kD * velocityError;

          // V = I*R + omega_motor/Kv to produce the commanded torque current
          double motorVelocityRadPerSec =
              sim.getVelocityRadPerSec() * ShooterConstants.hoodGearRatio;
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
