package frc.robot.util.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A swerve setpoint containing both chassis speeds and individual module states
 *
 * @param chassisSpeeds the desired chassis speeds
 * @param moduleStates the module states that achieve the chassis speeds
 */
public record SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {}
