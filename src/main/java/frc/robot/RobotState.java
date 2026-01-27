package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized robot state estimation
 */
public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  // kalman filter pose estimator
  private final SwerveDrivePoseEstimator poseEstimator;

  // odometry-only pose (no vision corrections) for comparison/debugging
  @Getter @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @Getter private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  // last known state for delta calculation
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = Rotation2d.kZero;

  private RobotState() {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("RobotState/Velocity", robotVelocity);
  }

  /** returns the kalman filtered estimated pose */
  @AutoLogOutput
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** resets both odometry and the kalman filter to the given pose */
  public void resetPose(Pose2d pose) {
    odometryPose = pose;
    poseEstimator.resetPosition(lastGyroAngle, lastModulePositions, pose);
  }

  /**
   * Adds an odometry observation from the drive subsystem
   *
   * @param observation the odometry observation
   */
  public void addOdometryObservation(OdometryObservation observation) {
    // calculate module deltas for odometry-only pose
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

    // update odometry-only pose
    Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    twist =
        new Twist2d(
            twist.dx, twist.dy, lastGyroAngle.minus(odometryPose.getRotation()).getRadians());
    odometryPose = odometryPose.exp(twist);

    // update Kalman filter with odometry
    poseEstimator.updateWithTime(
        observation.timestamp(), lastGyroAngle, observation.wheelPositions());

    // calculate robot velocity from module deltas
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] =
          new SwerveModuleState(moduleDeltas[i].distanceMeters / 0.02, moduleDeltas[i].angle);
    }
    robotVelocity = kinematics.toChassisSpeeds(moduleStates);
  }

  /**
   * Adds a vision observation
   *
   * @param observation the vision observation
   */
  public void addVisionObservation(VisionObservation observation) {
    poseEstimator.addVisionMeasurement(
        observation.visionPose(), observation.timestamp(), observation.stdDevs());
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
