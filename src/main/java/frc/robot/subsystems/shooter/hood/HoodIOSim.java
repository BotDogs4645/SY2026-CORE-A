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
    sim.update(Constants.loopPeriodSecs);

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
      }
      case CLOSED_LOOP -> {
        // errors in rotations to match TalonFX PositionTorqueCurrentFOC units
        double positionError = Units.radiansToRotations(outputs.positionRad - sim.getAngleRads());
        double velocityError =
            Units.radiansToRotations(outputs.velocityRadsPerSec - sim.getVelocityRadPerSec());
        double torqueCurrent = outputs.kP * positionError + outputs.kD * velocityError;

        // convert torque current to voltage: V = I*R + omega_motor/Kv
        double motorVelocityRadPerSec = sim.getVelocityRadPerSec() * ShooterConstants.hoodGearRatio;
        appliedVolts =
            torqueCurrent * MOTOR.rOhms + motorVelocityRadPerSec / MOTOR.KvRadPerSecPerVolt;
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
      }
    }

    sim.setInputVoltage(appliedVolts);
  }
}
