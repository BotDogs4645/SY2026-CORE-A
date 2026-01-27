package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** utility class for epsilon comparisons */
public class EqualsUtil {
  public static final double kEpsilon = 1e-12;

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(Rotation2d a, Rotation2d b, double epsilon) {
    return epsilonEquals(a.getRadians(), b.getRadians(), epsilon);
  }

  public static boolean epsilonEquals(Rotation2d a, Rotation2d b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(Translation2d a, Translation2d b, double epsilon) {
    return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
  }

  public static boolean epsilonEquals(Translation2d a, Translation2d b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(Pose2d a, Pose2d b, double epsilon) {
    return epsilonEquals(a.getTranslation(), b.getTranslation(), epsilon)
        && epsilonEquals(a.getRotation(), b.getRotation(), epsilon);
  }

  public static boolean epsilonEquals(Pose2d a, Pose2d b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(Twist2d a, Twist2d b, double epsilon) {
    return epsilonEquals(a.dx, b.dx, epsilon)
        && epsilonEquals(a.dy, b.dy, epsilon)
        && epsilonEquals(a.dtheta, b.dtheta, epsilon);
  }

  public static boolean epsilonEquals(Twist2d a, Twist2d b) {
    return epsilonEquals(a, b, kEpsilon);
  }
}
