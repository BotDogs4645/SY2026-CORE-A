package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.12);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.05);

  private static final LoggedTunableNumber shotDetectionTolerance =
      new LoggedTunableNumber("Flywheel/shotDetectionToleranceRadPerSec", 20.0);
  private static final LoggedTunableNumber atGoalTolerance =
      new LoggedTunableNumber("Flywheel/atGoalToleranceRadPerSec", 10.0);
  private static final LoggedTunableNumber atGoalDebounceSecs =
      new LoggedTunableNumber("Flywheel/atGoalDebounceSecs", 0.2);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIO.FlywheelIOOutputs outputs = new FlywheelIO.FlywheelIOOutputs();

  // sim shot injection
  private static final int SIM_SHOT_INTERVAL_CYCLES = 60; // ~1 se between shots
  private int simShotCycleCounter = 0;

  private double goalVelocityRadPerSec = 0.0;
  /** -- GETTER -- returns the number of detected shots */
  @Getter private int shotCount = 0;

  private int prevShotCount = 0;
  private boolean isAtSpeed = false;

  private final Debouncer atGoalDebouncer;

  private final Alert disconnectedAlert =
      new Alert("Flywheel motor disconnected!", AlertType.kWarning);

  public Flywheel(FlywheelIO io) {
    this.io = io;
    atGoalDebouncer = new Debouncer(atGoalDebounceSecs.get(), Debouncer.DebounceType.kRising);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    disconnectedAlert.set(!inputs.connected);

    Logger.recordOutput("Flywheel/goalVelocityRadPerSec", goalVelocityRadPerSec);
    Logger.recordOutput("Flywheel/shotCount", shotCount);
  }

  @Override
  public void periodicAfterScheduler() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> new Debouncer(atGoalDebounceSecs.get(), Debouncer.DebounceType.kRising),
        atGoalDebounceSecs);

    if (goalVelocityRadPerSec == 0.0) {
      outputs.mode = FlywheelIO.FlywheelIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      isAtSpeed = false;
    } else {
      outputs.mode = FlywheelIO.FlywheelIOOutputMode.CLOSED_LOOP;
      outputs.velocityRadsPerSec = goalVelocityRadPerSec;

      outputs.kV = kV.get();
      outputs.kP = kP.get();

      // shot detection logic
      double error = goalVelocityRadPerSec - inputs.velocityRadsPerSec;

      if (isAtSpeed) {
        // we were at speed, did we drop velocity suddenly?
        if (error > shotDetectionTolerance.get()) {
          shotCount++;
          isAtSpeed = false;
        }
      } else {
        // we are recovering or spinning up
        if (Math.abs(error) < atGoalTolerance.get()) {
          isAtSpeed = true;
        }
      }

      // sim only
      simShotCycleCounter++;
      if (simShotCycleCounter >= SIM_SHOT_INTERVAL_CYCLES && isAtSpeed) {
        io.simulateShotDisturbance();
        simShotCycleCounter = 0;
      }
    }

    io.applyOutputs(outputs);
  }

  /** sets the flywheel goal velocity and begins spin-up */
  public void setGoal(double velocityRadPerSec) {
    goalVelocityRadPerSec = velocityRadPerSec;
  }

  /** returns true if the flywheel is at its goal velocity (debounced!) */
  @AutoLogOutput(key = "Flywheel/atGoal")
  public boolean atGoal() {
    if (goalVelocityRadPerSec == 0.0) return false;
    return atGoalDebouncer.calculate(
        Math.abs(inputs.velocityRadsPerSec - goalVelocityRadPerSec) < atGoalTolerance.get());
  }

  /** returns the current flywheel velocity in rad/s */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  /** returns a trigger that fires once per detected shot */
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
          setGoal(0.0);
        },
        this);
  }
}
