package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit and robot/wheel configuration. Change
 * robotType and wheelType to switch between different hardware configurations.
 */
public final class Constants {
  // da da da da da da da da da da da
  // CHANGE THESE TO THE CORRECT VALUE FOR THE ROBOT
  // DO NOT FORGET
  private static RobotType robotType = RobotType.SIMBOT;
  private static WheelType wheelType = WheelType.BILLET;
  // da da da da da da da da da da da

  // endgame alert times
  public static final double ENDGAME_ALERT_1_TIME = 30.0;
  public static final double ENDGAME_ALERT_2_TIME = 15.0;

  // battery monitoring
  public static final double LOW_BATTERY_VOLTAGE = 11.5;
  public static final double LOW_BATTERY_DISABLED_TIME = 1.5; // seconds

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static WheelType getWheels() {
    return wheelType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT, // Simulation only
    DEVBOT, // Practice/dev robot
    COMPBOT // Competition robot
  }

  public enum WheelType {
    SPIKE_GRIP, // Spike Grip wheels
    BILLET // Billet wheels
  }

  public class TurretConstants {
    public static final int ROTATION_MOTOR_ID = 5;
    public static final double ROTATION_GEAR_RATIO = 2.5;
    public static final double ROTATION_kP = 20;
    public static final double ROTATION_kI = 0.5;
    public static final double ROTATION_kD = 4;
  }
}
