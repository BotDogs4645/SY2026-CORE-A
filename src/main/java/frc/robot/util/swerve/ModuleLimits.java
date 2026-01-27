package frc.robot.util.swerve;

/**
 * Limits for swerve module motion used by SwerveSetpointGenerator
 *
 * @param maxDriveVelocity maximum drive wheel velocity in m/s
 * @param maxDriveAcceleration maximum drive wheel acceleration in m/s^2
 * @param maxSteeringVelocity maximum steering velocity in rad/s
 */
public record ModuleLimits(
    double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}
