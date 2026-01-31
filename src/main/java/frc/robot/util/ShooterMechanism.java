package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/** Logs the shooter turret aim direction as a Pose3d for the 3D field in AdvantageScope. */
public class ShooterMechanism {

  public void update(
      Pose2d robotPose,
      double turretAngleRad,
      double hoodAngleRad,
      double flywheelVelocity,
      boolean flywheelAtGoal) {
    double fieldYaw = robotPose.getRotation().getRadians() + turretAngleRad;
    Logger.recordOutput(
        "Shooter/TurretAim",
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            ShooterConstants.turretZOffsetMeters,
            new Rotation3d(0.0, -hoodAngleRad, fieldYaw)));
  }
}
