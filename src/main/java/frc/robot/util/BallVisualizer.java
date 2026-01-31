package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** static utility class that animates ball game pieces flying from the launcher to the hub */
public class BallVisualizer {
  private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  private static DoubleSupplier turretAngleSupplier = () -> 0.0;
  private static DoubleSupplier hoodAngleSupplier = () -> 0.0;

  private static final List<ShotAnimation> activeShots = new ArrayList<>();
  private static int ballCount = 8;

  private BallVisualizer() {}

  /** sets the robot pose supplier for computing launch positions */
  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  /** sets the shooter state suppliers for computing launch direction */
  public static void setShooterStateSuppliers(
      DoubleSupplier turretAngle, DoubleSupplier hoodAngle) {
    turretAngleSupplier = turretAngle;
    hoodAngleSupplier = hoodAngle;
  }

  /** sets the number of balls in the hopper */
  public static void setBallCount(int count) {
    ballCount = count;
  }

  /** returns the number of balls remaining in the hopper */
  public static int getBallCount() {
    return ballCount;
  }

  /** logs a held ball at the shooter muzzle if any balls remain */
  public static void showHeldBall() {
    if (ballCount > 0) {
      Pose2d robotPose = robotPoseSupplier.get();
      double heading = robotPose.getRotation().getRadians();
      double turretAngle = turretAngleSupplier.getAsDouble();
      double hoodAngle = hoodAngleSupplier.getAsDouble();

      double pivotX = robotPose.getX() + ShooterConstants.turretXOffsetMeters * Math.cos(heading);
      double pivotY = robotPose.getY() + ShooterConstants.turretXOffsetMeters * Math.sin(heading);
      double pivotZ = ShooterConstants.turretZOffsetMeters;

      double globalTurretAngle = heading + turretAngle;
      double horizontalExtension = ShooterConstants.hoodArmLengthMeters * Math.cos(hoodAngle);
      double verticalExtension = ShooterConstants.hoodArmLengthMeters * Math.sin(hoodAngle);

      Pose3d muzzle =
          new Pose3d(
              pivotX + horizontalExtension * Math.cos(globalTurretAngle),
              pivotY + horizontalExtension * Math.sin(globalTurretAngle),
              pivotZ + verticalExtension,
              new Rotation3d());
      Logger.recordOutput("BallVisualizer/HeldBalls", new Pose3d[] {muzzle});
    } else {
      Logger.recordOutput("BallVisualizer/HeldBalls", new Pose3d[0]);
    }
  }

  /** returns a command that animates a ball shot from the current robot/shooter state to the hub */
  public static Command shoot() {
    return new ScheduleCommand(
        Commands.defer(
            () -> {
              // capture state at fire time
              Pose2d robotPose = robotPoseSupplier.get();
              double turretAngle = turretAngleSupplier.getAsDouble();
              double hoodAngle = hoodAngleSupplier.getAsDouble();

              // compute 3D launch position from robot pose info
              double robotX = robotPose.getX();
              double robotY = robotPose.getY();
              double robotHeading = robotPose.getRotation().getRadians();

              // turret pivot offset
              double pivotX =
                  robotX + ShooterConstants.turretXOffsetMeters * Math.cos(robotHeading);
              double pivotY =
                  robotY + ShooterConstants.turretXOffsetMeters * Math.sin(robotHeading);
              double pivotZ = ShooterConstants.turretZOffsetMeters;

              // hood arm extension from turret pivot
              double globalTurretAngle = robotHeading + turretAngle;
              double horizontalExtension =
                  ShooterConstants.hoodArmLengthMeters * Math.cos(hoodAngle);
              double verticalExtension = ShooterConstants.hoodArmLengthMeters * Math.sin(hoodAngle);

              Translation3d launchPosition =
                  new Translation3d(
                      pivotX + horizontalExtension * Math.cos(globalTurretAngle),
                      pivotY + horizontalExtension * Math.sin(globalTurretAngle),
                      pivotZ + verticalExtension);

              // determine target based on alliance
              Translation3d target;
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              target =
                  isRed ? FieldConstants.Hub.oppTopCenterPoint : FieldConstants.Hub.topCenterPoint;

              // decrement ball count
              if (ballCount > 0) {
                ballCount--;
              }

              // add tracer for the full trajectory arc
              ShotTracer.addTracer(launchPosition, target);

              // create animation
              double startTime = Timer.getFPGATimestamp();
              double duration = ShooterConstants.shotFlightDurationSecs;
              Translation3d finalLaunch = launchPosition;
              Translation3d finalTarget = target;

              activeShots.add(new ShotAnimation(finalLaunch, finalTarget, startTime, duration));

              return Commands.waitSeconds(duration);
            },
            Commands.none().getRequirements()));
  }

  /** logs all active ball positions */
  public static void periodic() {
    double now = Timer.getFPGATimestamp();
    Iterator<ShotAnimation> it = activeShots.iterator();
    List<Pose3d> ballPositions = new ArrayList<>();

    while (it.hasNext()) {
      ShotAnimation shot = it.next();
      double elapsed = now - shot.startTime;
      double t = elapsed / shot.duration;

      if (t > 1.0) {
        it.remove();
        continue;
      }

      double x = lerp(shot.launch.getX(), shot.target.getX(), t);
      double y = lerp(shot.launch.getY(), shot.target.getY(), t);
      double zLinear = lerp(shot.launch.getZ(), shot.target.getZ(), t);

      double arcHeight =
          Math.max(shot.launch.getZ(), shot.target.getZ()) * 0.5
              + shot.launch.getDistance(shot.target) * 0.15;
      double zArc = 4.0 * arcHeight * t * (1.0 - t);

      ballPositions.add(new Pose3d(x, y, zLinear + zArc, new Rotation3d()));
    }

    Logger.recordOutput("BallVisualizer/ShotBalls", ballPositions.toArray(new Pose3d[0]));
    Logger.recordOutput("BallVisualizer/BallCount", ballCount);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private record ShotAnimation(
      Translation3d launch, Translation3d target, double startTime, double duration) {}
}
