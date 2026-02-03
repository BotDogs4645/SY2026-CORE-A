// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private TalonFX turretRotationMotor;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, Constants.TurretConstants.ROTATION_GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  /** Creates a new Turret. */
  public Turret() {
    turretRotationMotor = new TalonFX(Constants.TurretConstants.ROTATION_MOTOR_ID, "CANivore");
    turretRotationMotor.setPosition(0);

    Slot0Configs configs =
        new Slot0Configs()
            .withKP(Constants.TurretConstants.ROTATION_kP)
            .withKI(Constants.TurretConstants.ROTATION_kI)
            .withKD(Constants.TurretConstants.ROTATION_kD)
            .withKA(0.001);
    turretRotationMotor.getConfigurator().apply(configs);

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = 1;
    turretRotationMotor.getConfigurator().apply(feedbackConfigs);

    var talonFXSim = turretRotationMotor.getSimState();
    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Turret/position", turretRotationMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Turret/velocity", turretRotationMotor.getVelocity().getValueAsDouble());

    if (!Robot.isReal()) {
      var talonFXSim = turretRotationMotor.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltageMeasure();

      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
      m_motorSimModel.update(0.020); // assume 20 ms loop time

      // apply the new rotor position and velocity to the TalonFX;
      // note that this is rotor position/velocity (before gear ratio), but
      // DCMotorSim returns mechanism position/velocity (after gear ratio)
      talonFXSim.setRawRotorPosition(
          m_motorSimModel
              .getAngularPosition()
              .times(Constants.TurretConstants.ROTATION_GEAR_RATIO));
      talonFXSim.setRotorVelocity(
          m_motorSimModel
              .getAngularVelocity()
              .times(Constants.TurretConstants.ROTATION_GEAR_RATIO));
    }
  }

  public void startTurret() {
    turretRotationMotor.set(0.5);
  }

  public void stopTurret() {
    turretRotationMotor.set(0);
  }

  public void setControl(ControlRequest control) {
    turretRotationMotor.setControl(control);
  }
}
