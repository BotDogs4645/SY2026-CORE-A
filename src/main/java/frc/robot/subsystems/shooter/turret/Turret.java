package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  // TODO: physical limits
  public static final double minAngleRad = Units.degreesToRadians(-177.5);
  public static final double maxAngleRad = Units.degreesToRadians(177.5);

  // tunable gains
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.0);
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Turret/maxVelocityRadPerSec", 2.0 * Math.PI);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Turret/maxAccelerationRadPerSecSq", 4.0 * Math.PI);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Turret/toleranceDeg", 1.0);

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO.TurretIOOutputs outputs = new TurretIO.TurretIOOutputs();

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private double positionOffset = 0.0;

  private final Alert disconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kWarning);

  public Turret(TurretIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    disconnectedAlert.set(!inputs.motorConnected);

    Logger.recordOutput("Turret/positionDeg", Units.radiansToDegrees(getPosition()));
    Logger.recordOutput("Turret/goalDeg", Units.radiansToDegrees(goal.position));
    Logger.recordOutput("Turret/setpointPositionDeg", Units.radiansToDegrees(setpoint.position));
    Logger.recordOutput(
        "Turret/setpointVelocityDegPerSec", Units.radiansToDegrees(setpoint.velocity));
  }

  @Override
  public void periodicAfterScheduler() {
    // update profile constraints if tunable values changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);

    // profile for closed-loop mode
    if (outputs.mode == TurretIO.TurretIOOutputMode.CLOSED_LOOP) {
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goal);
      outputs.position = setpoint.position - positionOffset;
      outputs.velocity = setpoint.velocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }

    io.applyOutputs(outputs);
  }

  /** returns the current turret position in radians */
  @AutoLogOutput(key = "Turret/measuredPositionRad")
  public double getPosition() {
    return inputs.positionRads + positionOffset;
  }

  /** sets the zero offset so that the current position reads as the endstop angle */
  public void zero() {
    positionOffset = minAngleRad - inputs.positionRads;
    setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
    goal = new TrapezoidProfile.State(getPosition(), 0.0);
  }

  /** sets the goal angle in radians, includes legal angle checking */
  public void setGoal(double angleRad) {
    double wrapped = findClosestLegalAngle(angleRad, getPosition());
    goal = new TrapezoidProfile.State(wrapped, 0.0);
    outputs.mode = TurretIO.TurretIOOutputMode.CLOSED_LOOP;
  }

  /**
   * finds the closest angle equivalent to targetRad (mod 2pi) that is within the legal range and
   * closest to currentRad
   */
  private double findClosestLegalAngle(double targetRad, double currentRad) {
    double normalized = MathUtil.angleModulus(targetRad);
    double bestAngle = normalized;
    double bestDistance = Double.MAX_VALUE;
    boolean foundLegal = false;

    for (int i = -1; i <= 1; i++) {
      double candidate = normalized + i * 2.0 * Math.PI;
      if (candidate >= minAngleRad && candidate <= maxAngleRad) {
        double distance = Math.abs(candidate - currentRad);
        if (!foundLegal || distance < bestDistance) {
          bestAngle = candidate;
          bestDistance = distance;
          foundLegal = true;
        }
      }
    }

    if (!foundLegal) {
      bestAngle = MathUtil.clamp(normalized, minAngleRad, maxAngleRad);
    }

    return bestAngle;
  }

  /** returns true if the turret is at its goal position */
  @AutoLogOutput(key = "Turret/atGoal")
  public boolean atGoal() {
    return Math.abs(getPosition() - goal.position) < Units.degreesToRadians(toleranceDeg.get());
  }

  /** command to move to a fixed angle in radians */
  public Command runFixedCommand(double angleRad) {
    return Commands.runOnce(() -> setGoal(angleRad), this).andThen(Commands.idle(this));
  }

  /** command to continuously track the alliance hub center using the robot pose */
  public Command trackHubCommand(Supplier<Pose2d> robotPoseSupplier) {
    return Commands.run(
            () -> {
              Pose2d pose = robotPoseSupplier.get();

              // pick hub based on alliance
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              Translation2d hubCenter =
                  isRed
                      ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
                      : FieldConstants.Hub.topCenterPoint.toTranslation2d();

              // field-relative angle from robot to hub
              double dx = hubCenter.getX() - pose.getX();
              double dy = hubCenter.getY() - pose.getY();
              double fieldAngleToHub = Math.atan2(dy, dx);

              // convert to turret-relative angle (turret 0 = robot forward)
              double turretAngle = fieldAngleToHub - pose.getRotation().getRadians();

              setGoal(turretAngle);
            },
            this)
        .withName("TrackHub");
  }

  /** command to zero the turret at the endstop */
  public Command zeroCommand() {
    return Commands.runOnce(this::zero, this);
  }

  /** command to stop the turret */
  public Command stopCommand() {
    return Commands.runOnce(
        () -> {
          outputs.mode = TurretIO.TurretIOOutputMode.BRAKE;
          setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
          goal = new TrapezoidProfile.State(getPosition(), 0.0);
        },
        this);
  }
}
