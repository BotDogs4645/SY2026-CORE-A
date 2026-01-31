package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Gemini is the physics genius... we will see how this works -CR
 * Stateless auto-shot calculator that models projectile motion with gravity and air drag,
 * compensates for robot velocity, and iteratively solves for turret yaw, hood pitch, and flywheel
 * speed.
 */
public class AutoShotCalculator {

  public record ShotSolution(
      double turretAngleRad,
      double hoodAngleRad,
      double flywheelVelocityRadPerSec,
      double distanceToTargetMeters,
      boolean isSolutionFound) {
    public static ShotSolution none() {
      return new ShotSolution(0.0, 0.0, 0.0, 0.0, false);
    }
  }

  // ball properties
  private static final LoggedTunableNumber ballMassKg =
      new LoggedTunableNumber("AutoShot/ballMassKg", 0.215);
  private static final LoggedTunableNumber ballDiameterMeters =
      new LoggedTunableNumber("AutoShot/ballDiameterMeters", 0.150114);
  private static final LoggedTunableNumber dragCoefficient =
      new LoggedTunableNumber("AutoShot/dragCoefficient", 0.5);
  private static final LoggedTunableNumber airDensityKgPerM3 =
      new LoggedTunableNumber("AutoShot/airDensityKgPerM3", 1.225);

  // flywheel
  private static final LoggedTunableNumber flywheelSpeedDropRadPerSec =
      new LoggedTunableNumber("AutoShot/flywheelSpeedDropRadPerSec", 42.0);
  private static final LoggedTunableNumber launchEfficiency =
      new LoggedTunableNumber("AutoShot/launchEfficiency", 0.95);

  // simulation
  private static final LoggedTunableNumber simTimeStepSecs =
      new LoggedTunableNumber("AutoShot/simTimeStepSecs", 0.005);

  // angle sweep
  private static final LoggedTunableNumber minLaunchAngleDeg =
      new LoggedTunableNumber("AutoShot/minLaunchAngleDeg", 20.0);
  private static final LoggedTunableNumber maxLaunchAngleDeg =
      new LoggedTunableNumber("AutoShot/maxLaunchAngleDeg", 60.0);
  private static final LoggedTunableNumber angleStepDeg =
      new LoggedTunableNumber("AutoShot/angleStepDeg", 0.5);

  // speed limit
  private static final LoggedTunableNumber maxLaunchSpeedMps =
      new LoggedTunableNumber("AutoShot/maxLaunchSpeedMps", 50.0);

  // trajectory visualization
  private static final int TRAJECTORY_LINE_POINTS = 50;
  private static final Pose3d[] EMPTY_TRAJECTORY = new Pose3d[0];

  private static final double GRAVITY = 9.81;
  private static final int BINARY_SEARCH_ITERATIONS = 20;

  /**
   * Calculate the optimal shot solution given the current robot state and target position.
   *
   * @param robotPose current robot pose on the field
   * @param robotRelativeSpeeds robot-relative chassis speeds
   * @param targetPosition 3D position of the target on the field
   * @return a ShotSolution with turret angle, hood angle, flywheel speed, and validity
   */
  public ShotSolution calculate(
      Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation3d targetPosition) {

    // 1. Compute shooter pivot position from robot pose + offsets
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double heading = robotPose.getRotation().getRadians();
    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);

    // turret pivot in field coords
    double pivotX = robotX + ShooterConstants.turretXOffsetMeters * cosH;
    double pivotY = robotY + ShooterConstants.turretXOffsetMeters * sinH;
    double pivotZ = ShooterConstants.turretZOffsetMeters;

    // 2. Compute 3D delta from pivot to target
    double dx = targetPosition.getX() - pivotX;
    double dy = targetPosition.getY() - pivotY;
    double dz = targetPosition.getZ() - pivotZ;

    // 3. Convert robot-relative speeds to field-relative
    double fieldVx =
        robotRelativeSpeeds.vxMetersPerSecond * cosH - robotRelativeSpeeds.vyMetersPerSecond * sinH;
    double fieldVy =
        robotRelativeSpeeds.vxMetersPerSecond * sinH + robotRelativeSpeeds.vyMetersPerSecond * cosH;

    // 4. Estimate flight time from horizontal distance / rough speed estimate
    double horizontalDist = Math.hypot(dx, dy);
    double roughFlightTime = estimateFlightTime(horizontalDist);

    // 5. Adjust target by -fieldVelocity * flightTime (lead the shot)
    double effectiveDx = dx - fieldVx * roughFlightTime;
    double effectiveDy = dy - fieldVy * roughFlightTime;

    // 6. Extract effective 2D distance and height delta
    double effectiveDist = Math.hypot(effectiveDx, effectiveDy);
    double targetHeight = dz;

    // field yaw to effective target
    double fieldYaw = Math.atan2(effectiveDy, effectiveDx);

    // 7. Sweep all launch angles and find the one with the minimum required speed.
    double minAngleRad = Units.degreesToRadians(minLaunchAngleDeg.get());
    double maxAngleRad = Units.degreesToRadians(maxLaunchAngleDeg.get());
    double stepRad = Units.degreesToRadians(angleStepDeg.get());

    ShotSolution bestSolution = ShotSolution.none();
    double minFoundSpeed = Double.POSITIVE_INFINITY;

    for (double pitchRad = minAngleRad; pitchRad <= maxAngleRad; pitchRad += stepRad) {
      double speed = solveForSpeed(pitchRad, effectiveDist, targetHeight);
      if (speed > 0.0 && speed <= maxLaunchSpeedMps.get()) {
        if (speed < minFoundSpeed) {
          minFoundSpeed = speed;
          double turretAngle = fieldYaw - heading;
          double flywheelRadPerSec = exitSpeedToFlywheelVelocity(speed);
          bestSolution =
              new ShotSolution(turretAngle, pitchRad, flywheelRadPerSec, horizontalDist, true);
        }
      }
    }

    // 8. If a best solution was found, log it and return it.
    if (bestSolution.isSolutionFound()) {
      // Generate trajectory for the chosen best solution for logging
      double launchSpeed = flywheelVelocityToExitSpeed(bestSolution.flywheelVelocityRadPerSec());

      Pose3d[] linePoses =
          generateTrajectory(
              launchSpeed,
              bestSolution.hoodAngleRad(),
              fieldYaw,
              pivotX,
              pivotY,
              pivotZ,
              targetHeight);
      Logger.recordOutput("AutoShot/distanceMeters", horizontalDist);
      Logger.recordOutput("AutoShot/effectiveDistanceMeters", effectiveDist);
      Logger.recordOutput(
          "AutoShot/calculatedPitchDeg", Units.radiansToDegrees(bestSolution.hoodAngleRad()));
      Logger.recordOutput(
          "AutoShot/calculatedFlywheelRadPerSec", bestSolution.flywheelVelocityRadPerSec());
      Logger.recordOutput("AutoShot/solutionFound", true);
      Logger.recordOutput("AutoShot/TrajectoryLine", linePoses);

      return bestSolution;
    }

    // 9. No solution found
    Logger.recordOutput("AutoShot/distanceMeters", horizontalDist);
    Logger.recordOutput("AutoShot/effectiveDistanceMeters", effectiveDist);
    Logger.recordOutput("AutoShot/calculatedPitchDeg", 0.0);
    Logger.recordOutput("AutoShot/calculatedFlywheelRadPerSec", 0.0);
    Logger.recordOutput("AutoShot/solutionFound", false);
    Logger.recordOutput("AutoShot/TrajectoryLine", EMPTY_TRAJECTORY);

    return ShotSolution.none();
  }

  /**
   * Simulate the trajectory at a given launch speed and pitch angle using Euler integration with
   * drag. Returns the horizontal distance reached when the projectile passes through the target
   * height, or -1 if it never reaches that height.
   */
  private double simulateTrajectory(double launchSpeed, double pitchRad, double targetHeight) {
    double dt = simTimeStepSecs.get();
    double mass = ballMassKg.get();
    double radius = ballDiameterMeters.get() / 2.0;
    double area = Math.PI * radius * radius;
    double cd = dragCoefficient.get();
    double rho = airDensityKgPerM3.get();

    // drag constant: 0.5 * rho * Cd * A
    double dragK = 0.5 * rho * cd * area;

    // initial conditions (horizontal = x, vertical = z)
    double vx = launchSpeed * Math.cos(pitchRad);
    double vz = launchSpeed * Math.sin(pitchRad);
    double x = 0.0;
    double z = 0.0;

    double prevZ = z;

    // simulate up to 5 seconds of flight
    int maxSteps = (int) (5.0 / dt);
    for (int i = 0; i < maxSteps; i++) {
      double speed = Math.sqrt(vx * vx + vz * vz);
      double dragForce = dragK * speed * speed;
      double ax = -(dragForce / mass) * (vx / (speed + 1e-9));
      double az = -GRAVITY - (dragForce / mass) * (vz / (speed + 1e-9));

      vx += ax * dt;
      vz += az * dt;
      x += vx * dt;
      z += vz * dt;

      // check if we crossed the target height on the way down
      if (prevZ >= targetHeight && z <= targetHeight && vz <= 0) {
        // interpolate for more accuracy
        double frac = (prevZ - targetHeight) / (prevZ - z + 1e-9);
        return x - vx * dt * (1.0 - frac);
      }

      // bail early if projectile is below target and heading down
      if (z < targetHeight - 1.0 && vz < 0) {
        return -1.0;
      }

      prevZ = z;
    }

    return -1.0;
  }

  /**
   * Binary search for the launch speed that makes the projectile land at the target distance for a
   * given pitch angle and target height.
   *
   * @return required launch speed in m/s, or -1 if no valid speed found
   */
  private double solveForSpeed(double pitchRad, double targetDistance, double targetHeight) {
    double lo = 1.0;
    double hi = maxLaunchSpeedMps.get();

    // check if solution is even possible at max speed
    double maxDist = simulateTrajectory(hi, pitchRad, targetHeight);
    if (maxDist < targetDistance) {
      return -1.0;
    }

    for (int i = 0; i < BINARY_SEARCH_ITERATIONS; i++) {
      double mid = (lo + hi) / 2.0;
      double dist = simulateTrajectory(mid, pitchRad, targetHeight);
      if (dist < 0 || dist < targetDistance) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    // verify solution is close enough (within 5cm)
    double finalDist = simulateTrajectory((lo + hi) / 2.0, pitchRad, targetHeight);
    if (Math.abs(finalDist - targetDistance) > 0.05) {
      return -1.0;
    }

    return (lo + hi) / 2.0;
  }

  /**
   * Convert ball exit speed to flywheel angular velocity in rad/s. Accounts for launch efficiency
   * and flywheel speed drop during shot.
   */
  private double exitSpeedToFlywheelVelocity(double speedMps) {
    double wheelRadius = ShooterConstants.shooterWheelRadiusMeters;
    double efficiency = launchEfficiency.get();
    double surfaceSpeed = speedMps / efficiency;
    return (surfaceSpeed / wheelRadius) + flywheelSpeedDropRadPerSec.get();
  }

  /** Inverse of exitSpeedToFlywheelVelocity, used for re-creating launch speed for logging. */
  private double flywheelVelocityToExitSpeed(double flywheelRadPerSec) {
    double wheelRadius = ShooterConstants.shooterWheelRadiusMeters;
    double efficiency = launchEfficiency.get();
    double surfaceSpeed = (flywheelRadPerSec - flywheelSpeedDropRadPerSec.get()) * wheelRadius;
    return surfaceSpeed * efficiency;
  }

  /**
   * Run the Euler sim for the solved shot and return field-space Pose3d points along the arc. Used
   * for AdvantageScope 3D trajectory visualization.
   */
  private Pose3d[] generateTrajectory(
      double launchSpeed,
      double pitchRad,
      double fieldYaw,
      double pivotX,
      double pivotY,
      double pivotZ,
      double targetHeight) {
    double dt = simTimeStepSecs.get();
    double mass = ballMassKg.get();
    double radius = ballDiameterMeters.get() / 2.0;
    double area = Math.PI * radius * radius;
    double dragK = 0.5 * airDensityKgPerM3.get() * dragCoefficient.get() * area;

    double cosYaw = Math.cos(fieldYaw);
    double sinYaw = Math.sin(fieldYaw);

    // sim state (2D: horizontal x, vertical z)
    double vx = launchSpeed * Math.cos(pitchRad);
    double vz = launchSpeed * Math.sin(pitchRad);
    double x = 0.0;
    double z = 0.0;

    // collect points until projectile crosses target height on the way down
    int maxSteps = (int) (5.0 / dt);
    // pre-count steps so we can size the array
    int totalSteps = 0;
    {
      double tvx = vx, tvz = vz, tx = 0.0, tz = 0.0, tpz = 0.0;
      for (int i = 0; i < maxSteps; i++) {
        double spd = Math.sqrt(tvx * tvx + tvz * tvz);
        double drag = dragK * spd * spd;
        tvx += (-(drag / mass) * (tvx / (spd + 1e-9))) * dt;
        tvz += (-GRAVITY - (drag / mass) * (tvz / (spd + 1e-9))) * dt;
        tx += tvx * dt;
        tz += tvz * dt;
        totalSteps++;
        if (tpz >= targetHeight && tz <= targetHeight && tvz <= 0) break;
        if (tz < targetHeight - 1.0 && tvz < 0) break;
        tpz = tz;
      }
    }

    // determine stride so we get ~TRAJECTORY_LINE_POINTS poses
    int stride = Math.max(1, totalSteps / TRAJECTORY_LINE_POINTS);
    int pointCount = (totalSteps / stride) + 1;
    Pose3d[] poses = new Pose3d[pointCount];
    int poseIdx = 0;

    double prevZ = 0.0;
    for (int i = 0; i < totalSteps; i++) {
      double spd = Math.sqrt(vx * vx + vz * vz);
      double drag = dragK * spd * spd;
      vx += (-(drag / mass) * (vx / (spd + 1e-9))) * dt;
      vz += (-GRAVITY - (drag / mass) * (vz / (spd + 1e-9))) * dt;
      x += vx * dt;
      z += vz * dt;

      if (i % stride == 0 && poseIdx < pointCount) {
        double fieldX = pivotX + x * cosYaw;
        double fieldY = pivotY + x * sinYaw;
        double fieldZ = pivotZ + z;
        poses[poseIdx++] = new Pose3d(fieldX, fieldY, fieldZ, new Rotation3d());
      }

      if (prevZ >= targetHeight && z <= targetHeight && vz <= 0) break;
      if (z < targetHeight - 1.0 && vz < 0) break;
      prevZ = z;
    }

    // trim if we exited early
    if (poseIdx < pointCount) {
      Pose3d[] trimmed = new Pose3d[poseIdx];
      System.arraycopy(poses, 0, trimmed, 0, poseIdx);
      return trimmed;
    }
    return poses;
  }

  /** Rough flight time estimate for velocity compensation. */
  private double estimateFlightTime(double distance) {
    // assume ~20 m/s average speed as rough estimate
    return distance / 20.0;
  }
}
