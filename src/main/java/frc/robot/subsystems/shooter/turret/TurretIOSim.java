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
    sim.update(Constants.loopPeriodSecs);

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
      }
      case CLOSED_LOOP -> {
        // errors in rotations to match TalonFX PositionTorqueCurrentFOC units
        double positionError =
            Units.radiansToRotations(outputs.position - sim.getAngularPositionRad());
        double velocityError =
            Units.radiansToRotations(outputs.velocity - sim.getAngularVelocityRadPerSec());
        double torqueCurrent = outputs.kP * positionError + outputs.kD * velocityError;

        // convert torque current to voltage: V = I*R + oh muh guh/Kv
        double motorVelocityRadPerSec =
            sim.getAngularVelocityRadPerSec() * ShooterConstants.turretGearRatio;
        appliedVolts =
            torqueCurrent * MOTOR.rOhms + motorVelocityRadPerSec / MOTOR.KvRadPerSecPerVolt;
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
      }
    }

    sim.setInputVoltage(appliedVolts);
  }
}
