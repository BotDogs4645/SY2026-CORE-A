package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/** Robot-specific hardware configuration for the drive subsystem. */
public class DriveConstants {
  // CAN bus configuration
  public static final CANBus canBus =
      switch (Constants.getRobot()) {
        case COMPBOT, SIMBOT -> new CANBus("CANivore", "./logs/example.hoot");
        case DEVBOT -> new CANBus("CANivore", "./logs/example.hoot"); // TODO: Update for dev bot
      };

  // Frame dimensions (per robot)
  public static final double trackWidthX =
      switch (Constants.getRobot()) {
        case DEVBOT -> Units.inchesToMeters(20.75);
        case COMPBOT, SIMBOT -> Units.inchesToMeters(22.0);
      };

  public static final double trackWidthY = trackWidthX; // Square chassis

  // Module positions
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2), // FL
    new Translation2d(trackWidthX / 2, -trackWidthY / 2), // FR
    new Translation2d(-trackWidthX / 2, trackWidthY / 2), // BL
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2) // BR
  };

  // Pigeon 2 gyro ID
  public static final int pigeonId =
      switch (Constants.getRobot()) {
        case DEVBOT -> 3;
        case COMPBOT, SIMBOT -> 2;
      };

  // Gear ratios
  public static final double driveGearRatio = 5.2734375;
  public static final double steerGearRatio = 26.09090909090909;
  public static final double coupleRatio = 3.375;

  // Drive motor inversion (left vs right side)
  public static final boolean invertLeftSide = true;
  public static final boolean invertRightSide = false;

  public record ModuleConfig(
      int driveMotorId,
      int steerMotorId,
      int encoderId,
      Angle encoderOffset,
      boolean steerInverted,
      boolean encoderInverted) {}

  private static final ModuleConfig[] moduleConfigsComp = {
    // FL - from TunerConstants
    new ModuleConfig(13, 12, 14, Rotations.of(-0.04931640625), false, false),
    // FR
    new ModuleConfig(4, 3, 5, Rotations.of(0.026123046875), false, false),
    // BL
    new ModuleConfig(10, 9, 11, Rotations.of(-0.480224609375), false, false),
    // BR
    new ModuleConfig(7, 6, 8, Rotations.of(0.40380859375), false, false),
  };

  private static final ModuleConfig[] moduleConfigsDev = {
    // FL - TODO: Set actual dev bot values
    new ModuleConfig(13, 12, 14, Rotations.of(0.0), false, false),
    // FR
    new ModuleConfig(4, 3, 5, Rotations.of(0.0), false, false),
    // BL
    new ModuleConfig(10, 9, 11, Rotations.of(0.0), false, false),
    // BR
    new ModuleConfig(7, 6, 8, Rotations.of(0.0), false, false),
  };

  // Module configs (motor IDs, encoder offsets - per robot)
  public static final ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT, SIMBOT -> moduleConfigsComp;
        case DEVBOT -> moduleConfigsDev;
      };
}
