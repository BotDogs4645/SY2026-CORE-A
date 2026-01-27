package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Swerve setpoint generator based on/stolen from 254's 2023 implementation This generator
 * constrains module velocities and accelerations to prevent wheel slip and ensure smooth motion.
 *
 * <p>"The algorithm finds the maximum feasible interpolation factor between the current setpoint
 * and the desired setpoint that respects all constraints using a modified regula falsi root-finding
 * method".
 */
public class SwerveSetpointGenerator {
  private static final double kEpsilon = 1e-6;

  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] moduleLocations;

  /**
   * Creates a new SwerveSetpointGenerator
   *
   * @param kinematics the swerve drive kinematics
   * @param moduleLocations the module locations relative to the robot center
   */
  public SwerveSetpointGenerator(
      SwerveDriveKinematics kinematics, Translation2d[] moduleLocations) {
    this.kinematics = kinematics;
    this.moduleLocations = moduleLocations;
  }

  /**
   * Generates a new setpoint that respects all module constraints
   *
   * @param limits the module limits to enforce
   * @param prevSetpoint the previous setpoint
   * @param desiredSpeeds the desired chassis speeds
   * @param dt the time step in seconds
   * @return a new setpoint that respects all constraints
   */
  public SwerveSetpoint generateSetpoint(
      ModuleLimits limits, SwerveSetpoint prevSetpoint, ChassisSpeeds desiredSpeeds, double dt) {

    SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
    // make sure desiredState angles are continuous with prevSetpoint angles
    for (int i = 0; i < 4; i++) {
      desiredModuleStates[i].optimize(prevSetpoint.moduleStates()[i].angle);
    }

    // if we're not really moving just return desired states
    boolean allStopped = true;
    for (int i = 0; i < 4; i++) {
      if (Math.abs(prevSetpoint.moduleStates()[i].speedMetersPerSecond) > kEpsilon
          || Math.abs(desiredModuleStates[i].speedMetersPerSecond) > kEpsilon) {
        allStopped = false;
        break;
      }
    }
    if (allStopped) {
      return new SwerveSetpoint(desiredSpeeds, desiredModuleStates);
    }

    // find the maximum s value that satisfies all constraints
    double maxS = 1.0;

    for (int i = 0; i < 4; i++) {
      SwerveModuleState prevState = prevSetpoint.moduleStates()[i];
      SwerveModuleState desiredState = desiredModuleStates[i];

      // check steering velocity constraint
      double maxSteeringChange = limits.maxSteeringVelocity() * dt;
      double steeringChange = Math.abs(desiredState.angle.minus(prevState.angle).getRadians());

      if (steeringChange > maxSteeringChange + kEpsilon) {
        double steeringS = maxSteeringChange / steeringChange;
        maxS = Math.min(maxS, steeringS);
      }

      // check drive acceleration constraint
      double velocityChange =
          Math.abs(desiredState.speedMetersPerSecond - prevState.speedMetersPerSecond);
      double maxVelocityChange = limits.maxDriveAcceleration() * dt;

      if (velocityChange > maxVelocityChange + kEpsilon) {
        double driveS = maxVelocityChange / velocityChange;
        maxS = Math.min(maxS, driveS);
      }

      // check max drive velocity constraint
      if (Math.abs(desiredState.speedMetersPerSecond) > limits.maxDriveVelocity() + kEpsilon) {
        // scale down to max velocity
        double velocityS = limits.maxDriveVelocity() / Math.abs(desiredState.speedMetersPerSecond);
        maxS = Math.min(maxS, velocityS);
      }
    }

    // apply refined constraints using regula falsi
    maxS = findSteeringMaxS(prevSetpoint.moduleStates(), desiredModuleStates, limits, dt, maxS);
    maxS = findDriveMaxS(prevSetpoint.moduleStates(), desiredModuleStates, limits, dt, maxS);

    // interpolate between previous and desired based on max s
    ChassisSpeeds interpolatedSpeeds =
        interpolateChassisSpeeds(prevSetpoint.chassisSpeeds(), desiredSpeeds, maxS);

    SwerveModuleState[] interpolatedStates = kinematics.toSwerveModuleStates(interpolatedSpeeds);
    for (int i = 0; i < 4; i++) {
      interpolatedStates[i].optimize(prevSetpoint.moduleStates()[i].angle);

      // clamp velocities to limits
      interpolatedStates[i].speedMetersPerSecond =
          Math.max(
              -limits.maxDriveVelocity(),
              Math.min(limits.maxDriveVelocity(), interpolatedStates[i].speedMetersPerSecond));
    }

    return new SwerveSetpoint(interpolatedSpeeds, interpolatedStates);
  }

  /** finds the maximum s value that respects steering velocity constraints using regula falsi. */
  private double findSteeringMaxS(
      SwerveModuleState[] prevStates,
      SwerveModuleState[] desiredStates,
      ModuleLimits limits,
      double dt,
      double maxS) {

    double maxSteeringChange = limits.maxSteeringVelocity() * dt;

    for (int i = 0; i < 4; i++) {
      // does this module need limiting?
      double totalChange = Math.abs(desiredStates[i].angle.minus(prevStates[i].angle).getRadians());
      if (totalChange <= maxSteeringChange + kEpsilon) {
        continue;
      }

      // binary search for max valid s
      double low = 0.0;
      double high = maxS;

      for (int iteration = 0; iteration < 10; iteration++) {
        double mid = (low + high) / 2.0;
        Rotation2d interpolatedAngle = prevStates[i].angle.interpolate(desiredStates[i].angle, mid);
        double change = Math.abs(interpolatedAngle.minus(prevStates[i].angle).getRadians());

        if (change <= maxSteeringChange + kEpsilon) {
          low = mid;
        } else {
          high = mid;
        }
      }

      maxS = Math.min(maxS, low);
    }

    return maxS;
  }

  /** finds the maximum s value that respects drive acceleration constraints using regula falsi */
  private double findDriveMaxS(
      SwerveModuleState[] prevStates,
      SwerveModuleState[] desiredStates,
      ModuleLimits limits,
      double dt,
      double maxS) {

    double maxAccelChange = limits.maxDriveAcceleration() * dt;

    for (int i = 0; i < 4; i++) {
      double velocityChange =
          Math.abs(desiredStates[i].speedMetersPerSecond - prevStates[i].speedMetersPerSecond);
      if (velocityChange <= maxAccelChange + kEpsilon) {
        continue;
      }

      double low = 0.0;
      double high = maxS;

      for (int iteration = 0; iteration < 10; iteration++) {
        double mid = (low + high) / 2.0;
        double interpolatedVelocity =
            prevStates[i].speedMetersPerSecond
                + mid
                    * (desiredStates[i].speedMetersPerSecond - prevStates[i].speedMetersPerSecond);
        double change = Math.abs(interpolatedVelocity - prevStates[i].speedMetersPerSecond);

        if (change <= maxAccelChange + kEpsilon) {
          low = mid;
        } else {
          high = mid;
        }
      }

      maxS = Math.min(maxS, low);
    }

    return maxS;
  }

  /** interpolates between two ChassisSpeeds based on a scale factor */
  private ChassisSpeeds interpolateChassisSpeeds(ChassisSpeeds start, ChassisSpeeds end, double s) {
    return new ChassisSpeeds(
        start.vxMetersPerSecond + s * (end.vxMetersPerSecond - start.vxMetersPerSecond),
        start.vyMetersPerSecond + s * (end.vyMetersPerSecond - start.vyMetersPerSecond),
        start.omegaRadiansPerSecond
            + s * (end.omegaRadiansPerSecond - start.omegaRadiansPerSecond));
  }
}
