package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Geometry utilities for converting between WPILib types */
public class GeomUtil {
  /** creates a pure translating transform */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  /** creates a pure rotating transform */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  /** converts a Pose2d to a Transform2d to be used in a kinematic chain */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /** converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /** converts a ChassisSpeeds to a Twist2d by extracting two velocities and using dt */
  public static Twist2d toTwist2d(ChassisSpeeds speeds) {
    return new Twist2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /** creates a new Pose2d at the given position with no rotation */
  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  /** creates a Pose3d from a Pose2d (z=0, no roll/pitch) */
  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(pose);
  }

  /**
   * interpolates between two poses based on scale factor t
   *
   * @param lhs the start pose
   * @param rhs the end pose
   * @param t the interpolation factor (0.0 = start, 1.0 = end)
   * @return the interpolated pose
   */
  public static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {
    if (t <= 0) {
      return lhs;
    } else if (t >= 1) {
      return rhs;
    }
    Twist2d twist = lhs.log(rhs);
    Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
    return lhs.exp(scaled);
  }
}
