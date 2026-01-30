package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  // TODO: physical limits
  public static final double minAngleRad = Units.degreesToRadians(0.0);
  public static final double maxAngleRad = Units.degreesToRadians(60.0);

  // tunable gains
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0.0);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/toleranceDeg", 1.0);

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIO.HoodIOOutputs outputs = new HoodIO.HoodIOOutputs();

  private double goalPositionRad = 0.0;
  private double goalVelocityRadsPerSec = 0.0;
  private double positionOffset = 0.0;

  private final Alert disconnectedAlert = new Alert("Hood motor disconnected!", AlertType.kWarning);

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    disconnectedAlert.set(!inputs.motorConnected);

    Logger.recordOutput("Hood/positionDeg", Units.radiansToDegrees(getPosition()));
    Logger.recordOutput("Hood/goalDeg", Units.radiansToDegrees(goalPositionRad));
  }

  @Override
  public void periodicAfterScheduler() {
    if (outputs.mode == HoodIO.HoodIOOutputMode.CLOSED_LOOP) {
      // clamp goal to legal range
      double clampedGoal = MathUtil.clamp(goalPositionRad, minAngleRad, maxAngleRad);

      outputs.positionRad = clampedGoal - positionOffset;
      outputs.velocityRadsPerSec = goalVelocityRadsPerSec;
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }

    io.applyOutputs(outputs);
  }

  /** returns the current hood position in radians */
  @AutoLogOutput(key = "Hood/measuredPositionRad")
  public double getPosition() {
    return inputs.positionRads + positionOffset;
  }

  /** sets the zero offset so that the current position reads as minAngle */
  public void zero() {
    positionOffset = minAngleRad - inputs.positionRads;
    goalPositionRad = getPosition();
    goalVelocityRadsPerSec = 0.0;
  }

  /** sets the goal angle and velocit */
  public void setGoal(double angleRad, double velocityRadsPerSec) {
    goalPositionRad = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
    goalVelocityRadsPerSec = velocityRadsPerSec;
    outputs.mode = HoodIO.HoodIOOutputMode.CLOSED_LOOP;
  }

  /** returns true if the hood is at its goal position */
  @AutoLogOutput(key = "Hood/atGoal")
  public boolean atGoal() {
    return Math.abs(getPosition() - goalPositionRad) < Units.degreesToRadians(toleranceDeg.get());
  }

  /** command to move to a fixed angle with optional velocity feedforward */
  public Command runFixedCommand(double angleRad, double velocityRadsPerSec) {
    return Commands.runOnce(() -> setGoal(angleRad, velocityRadsPerSec), this)
        .andThen(Commands.idle(this));
  }

  /** command to zero the hood at the minimum angle */
  public Command zeroCommand() {
    return Commands.runOnce(this::zero, this);
  }

  /** command to stop the hood */
  public Command stopCommand() {
    return Commands.runOnce(
        () -> {
          outputs.mode = HoodIO.HoodIOOutputMode.BRAKE;
          goalPositionRad = getPosition();
          goalVelocityRadsPerSec = 0.0;
        },
        this);
  }
}
