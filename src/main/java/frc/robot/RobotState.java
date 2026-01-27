package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized robot state estimation. Handles odometry updates, vision fusion, and provides pose
 * estimates to all subsystems.
 */
public class RobotState extends VirtualSubsystem {
  // pose buffer for time-synced vision updates
  private static final double poseBufferSizeSeconds = 2.0; //seconds

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // pose estimation state
  @Getter @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @Getter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;
  @Getter private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  // last known module positions and gyro angle for delta calculation
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = Rotation2d.kZero;

  private RobotState() {
    // da
  }

  @Override
  public void periodic() {
    Logger.recordOutput("RobotState/Velocity", robotVelocity);
  }

  /** resets the odometry and estimated pose to the given pose */
  public void resetPose(Pose2d pose) {
    odometryPose = pose;
    estimatedPose = pose;
    poseBuffer.clear();
  }

  /**
   * Adds an odometry observation from the drive subsystem
   *
   * @param observation the odometry observation
   */
  public void addOdometryObservation(OdometryObservation observation) {
    // calculate module deltas
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      moduleDeltas[i] =
          new SwerveModulePosition(
              observation.wheelPositions()[i].distanceMeters
                  - lastModulePositions[i].distanceMeters,
              observation.wheelPositions()[i].angle);
      lastModulePositions[i] = observation.wheelPositions()[i];
    }

    // update gyro angle
    if (observation.gyroAngle().isPresent()) {
      lastGyroAngle = observation.gyroAngle().get();
    } else {
      // no gyro, something went wrong!
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      lastGyroAngle = lastGyroAngle.plus(new Rotation2d(twist.dtheta));
    }

    Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    twist =
        new Twist2d(
            twist.dx, twist.dy, lastGyroAngle.minus(odometryPose.getRotation()).getRadians());

    odometryPose = odometryPose.exp(twist);

    estimatedPose = estimatedPose.exp(twist);

    poseBuffer.addSample(observation.timestamp(), odometryPose);

    robotVelocity =
        kinematics.toChassisSpeeds(
            new edu.wpi.first.math.kinematics.SwerveModuleState[] {
              new edu.wpi.first.math.kinematics.SwerveModuleState(
                  observation.wheelPositions()[0].distanceMeters / 0.02,
                  observation.wheelPositions()[0].angle),
              new edu.wpi.first.math.kinematics.SwerveModuleState(
                  observation.wheelPositions()[1].distanceMeters / 0.02,
                  observation.wheelPositions()[1].angle),
              new edu.wpi.first.math.kinematics.SwerveModuleState(
                  observation.wheelPositions()[2].distanceMeters / 0.02,
                  observation.wheelPositions()[2].angle),
              new edu.wpi.first.math.kinematics.SwerveModuleState(
                  observation.wheelPositions()[3].distanceMeters / 0.02,
                  observation.wheelPositions()[3].angle)
            });
  }

  /**
   * Adds a vision observation to correct the pose estimate
   *
   * @param observation the vision observation
   */
  public void addVisionObservation(VisionObservation observation) {
    Optional<Pose2d> odometryAtTime = poseBuffer.getSample(observation.timestamp());
    if (odometryAtTime.isEmpty()) {
      return;
    }

    // calculate the transform from odometry at vision time to current odometry
    Pose2d odometryDelta = odometryAtTime.get().relativeTo(odometryPose);

    // calculate the vision pose projected to current time
    Pose2d visionPoseNow =
        observation
            .visionPose()
            .plus(
                new edu.wpi.first.math.geometry.Transform2d(
                    odometryDelta.getTranslation().unaryMinus(),
                    odometryDelta.getRotation().unaryMinus()));

    double translationWeight = 1.0 / (observation.stdDevs().get(0, 0) + 0.01);
    double rotationWeight = 1.0 / (observation.stdDevs().get(2, 0) + 0.01);

    double odometryWeight = 10.0;
    double totalTranslationWeight = translationWeight + odometryWeight;
    double totalRotationWeight = rotationWeight + odometryWeight;

    // interpolate position
    Translation2d newTranslation =
        estimatedPose
            .getTranslation()
            .times(odometryWeight / totalTranslationWeight)
            .plus(visionPoseNow.getTranslation().times(translationWeight / totalTranslationWeight));

    // interpolate rotation
    Rotation2d newRotation =
        estimatedPose
            .getRotation()
            .interpolate(visionPoseNow.getRotation(), rotationWeight / totalRotationWeight);

    estimatedPose = new Pose2d(newTranslation, newRotation);
  }

  /** returns the current heading from the estimated pose */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  /** record for odometry observations from the drive subsystem */
  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  /** record for vision observations from camera subsystems */
  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
