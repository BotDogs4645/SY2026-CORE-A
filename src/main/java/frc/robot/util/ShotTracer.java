package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** static utility class that logs shot trajectory arcs as series of Pose3d points. */
public class ShotTracer {
  private static final List<Tracer> activeTracers = new ArrayList<>();

  private ShotTracer() {}

  /**
   * pre-computes and adds a trajectory tracer between launch and target positions
   *
   * @param launchPosition the 3D launch position
   * @param targetPosition the 3D target position
   */
  public static void addTracer(Translation3d launchPosition, Translation3d targetPosition) {
    int pointCount = ShooterConstants.shotTracerPointCount;
    Pose3d[] points = new Pose3d[pointCount];

    double arcHeight =
        Math.max(launchPosition.getZ(), targetPosition.getZ()) * 0.5
            + launchPosition.getDistance(targetPosition) * 0.15;

    for (int i = 0; i < pointCount; i++) {
      double t = (double) i / (pointCount - 1);
      double x = lerp(launchPosition.getX(), targetPosition.getX(), t);
      double y = lerp(launchPosition.getY(), targetPosition.getY(), t);
      double zLinear = lerp(launchPosition.getZ(), targetPosition.getZ(), t);
      double zArc = 4.0 * arcHeight * t * (1.0 - t);
      points[i] = new Pose3d(x, y, zLinear + zArc, new Rotation3d());
    }

    activeTracers.add(
        new Tracer(
            points, Timer.getFPGATimestamp() + ShooterConstants.shotFlightDurationSecs + 5.0));
  }

  /** logs all active tracers and removes expired ones. */
  public static void periodic() {
    double now = Timer.getFPGATimestamp();
    Iterator<Tracer> it = activeTracers.iterator();
    List<Pose3d> allPoints = new ArrayList<>();

    while (it.hasNext()) {
      Tracer tracer = it.next();
      if (now > tracer.expirationTimestamp) {
        it.remove();
      } else {
        for (Pose3d point : tracer.points) {
          allPoints.add(point);
        }
      }
    }

    Logger.recordOutput("ShotTracer", allPoints.toArray(new Pose3d[0]));
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private record Tracer(Pose3d[] points, double expirationTimestamp) {}
}
