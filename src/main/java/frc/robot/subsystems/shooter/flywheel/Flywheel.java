package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheel/torqueCurrentControlToleranceRadPerSec", 10.0);
  private static final LoggedTunableNumber torqueCurrentControlDebounceSecs =
      new LoggedTunableNumber("Flywheel/torqueCurrentControlDebounceSecs", 0.1);
  private static final LoggedTunableNumber atGoalDebounceSecs =
      new LoggedTunableNumber("Flywheel/atGoalDebounceSecs", 0.5);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIO.FlywheelIOOutputs outputs = new FlywheelIO.FlywheelIOOutputs();

  private double goalVelocityRadPerSec = 0.0;
  private int shotCount = 0;
  private int prevShotCount = 0;

  private final Debouncer torqueCurrentDebouncer;
  private final Debouncer atGoalDebouncer;
  private boolean inTorqueCurrentControl = false;

  private final Alert disconnectedAlert =
      new Alert("Flywheel motor disconnected!", AlertType.kWarning);

  public Flywheel(FlywheelIO io) {
    this.io = io;
    torqueCurrentDebouncer =
        new Debouncer(torqueCurrentControlDebounceSecs.get(), Debouncer.DebounceType.kRising);
    atGoalDebouncer = new Debouncer(atGoalDebounceSecs.get(), Debouncer.DebounceType.kRising);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    disconnectedAlert.set(!inputs.connected);

    Logger.recordOutput("Flywheel/goalVelocityRadPerSec", goalVelocityRadPerSec);
    Logger.recordOutput("Flywheel/shotCount", shotCount);
    Logger.recordOutput("Flywheel/inTorqueCurrentControl", inTorqueCurrentControl);
  }

  @Override
  public void periodicAfterScheduler() {
    if (outputs.mode != FlywheelIO.FlywheelIOOutputMode.COAST) {
      double error = Math.abs(goalVelocityRadPerSec - inputs.velocityRadsPerSec);

      // are we close enough to target for torque current mode transition>?
      boolean closeToTarget =
          torqueCurrentDebouncer.calculate(error < torqueCurrentControlTolerance.get());

      if (inTorqueCurrentControl) {
        if (!closeToTarget) {
          // shot detection based off velocity drop
          inTorqueCurrentControl = false;
          shotCount++;
          outputs.mode = FlywheelIO.FlywheelIOOutputMode.DUTY_CYCLE_BANG_BANG;
        } else {
          outputs.mode = FlywheelIO.FlywheelIOOutputMode.TORQUE_CURRENT_BANG_BANG;
        }
      } else {
        if (closeToTarget) {
          inTorqueCurrentControl = true;
          outputs.mode = FlywheelIO.FlywheelIOOutputMode.TORQUE_CURRENT_BANG_BANG;
        } else {
          outputs.mode = FlywheelIO.FlywheelIOOutputMode.DUTY_CYCLE_BANG_BANG;
        }
      }

      outputs.velocityRadsPerSec = goalVelocityRadPerSec;
    }

    io.applyOutputs(outputs);
  }

  /** sets the flywheel goal velocity and begins spin-up */
  public void setGoal(double velocityRadPerSec) {
    goalVelocityRadPerSec = velocityRadPerSec;
    inTorqueCurrentControl = false;
    torqueCurrentDebouncer.calculate(false);
    outputs.mode = FlywheelIO.FlywheelIOOutputMode.DUTY_CYCLE_BANG_BANG;
  }

  /** returns true if the flywheel is at its goal velocity (debounced!) */
  @AutoLogOutput(key = "Flywheel/atGoal")
  public boolean atGoal() {
    if (goalVelocityRadPerSec == 0.0) return false;
    return atGoalDebouncer.calculate(
        Math.abs(inputs.velocityRadsPerSec - goalVelocityRadPerSec)
            < torqueCurrentControlTolerance.get());
  }

  /** returns the current flywheel velocity in rad/s */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  /** returns the number of detected shots */
  @AutoLogOutput(key = "Flywheel/shotCount")
  public int getShotCount() {
    return shotCount;
  }

  /** returns a trigger that fires once per detected shot (rising edge on shotCount increment) */
  public Trigger shotDetectedTrigger() {
    return new Trigger(
        () -> {
          boolean fired = shotCount > prevShotCount;
          prevShotCount = shotCount;
          return fired;
        });
  }

  /** command to run the flywheel at a fixed velocity in rad/s */
  public Command runFixedCommand(double velocityRadPerSec) {
    return Commands.runOnce(() -> setGoal(velocityRadPerSec), this).andThen(Commands.idle(this));
  }

  /** command to stop the flywheel (coast mode) */
  public Command stopCommand() {
    return Commands.runOnce(
        () -> {
          goalVelocityRadPerSec = 0.0;
          inTorqueCurrentControl = false;
          outputs.mode = FlywheelIO.FlywheelIOOutputMode.COAST;
        },
        this);
  }
}
