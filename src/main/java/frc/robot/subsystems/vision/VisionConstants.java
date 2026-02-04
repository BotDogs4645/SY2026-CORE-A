package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout =
      FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();

  // Camera names, must match names configured on coprocessor
  public static final String camera0Name = "limelight";
  public static String camera1Name = "limelight-back";

  // Robot to camera transforms
  // These are not used by Limelight (check the webui) but are used for PhotonVision in simulation
  // mode.
  // So make sure these are correct even though they aren't used at all with Limelight.
  public static final Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, Units.degreesToRadians(30), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static final double maxAmbiguity = 0.6;
  public static final double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // QuestNav configuration
  public static final Transform3d robotToQuestTransform =
      new Transform3d(
          0.0,
          0.0,
          0.5, // x, y, z offset from robot center (meters)
          new Rotation3d(0.0, 0.0, 0.0) // roll, pitch, yaw
          );

  // QuestNav standard deviations
  public static final double questNavLinearStdDev = 0.02; // meters
  public static final double questNavAngularStdDev = 0.035; // radians (~2 deg)

  // multiplier to adjust QuestNav trust (>1 = less trust, <1 = more trust)
  public static final double questNavStdDevFactor = 1.0;
}
