package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Mechanism2d visualization for the shooter subsystem. Two separate canvases: a side-view for
 * hood/flywheel state, and a top-down view for turret yaw.
 */
public class ShooterMechanism {
  // Side view canvas - hood angle + flywheel state
  private final LoggedMechanism2d sideView = new LoggedMechanism2d(1.0, 1.5);
  private final LoggedMechanismLigament2d turretPost;
  private final LoggedMechanismLigament2d hoodArm;
  private final LoggedMechanismLigament2d flywheelIndicator;

  // Top-down canvas - turret yaw rotation
  private final LoggedMechanism2d topDown = new LoggedMechanism2d(1.0, 1.0);
  private final LoggedMechanismLigament2d turretPointer;
  private final LoggedMechanismLigament2d rangeLeft;
  private final LoggedMechanismLigament2d rangeRight;

  private static final Color8Bit GRAY = new Color8Bit(Color.kGray);
  private static final Color8Bit DARK_GRAY = new Color8Bit(Color.kDarkGray);
  private static final Color8Bit GREEN = new Color8Bit(Color.kGreen);
  private static final Color8Bit RED = new Color8Bit(Color.kRed);
  private static final Color8Bit CYAN = new Color8Bit(Color.kCyan);

  public ShooterMechanism() {
    // side view: turret post (vertical) -> hood arm -> flywheel indicator ---
    LoggedMechanismRoot2d sideRoot = sideView.getRoot("SideRoot", 0.5, 0.1);
    turretPost =
        sideRoot.append(
            new LoggedMechanismLigament2d(
                "TurretPost", ShooterConstants.turretZOffsetMeters * 2.0, 90.0, 6, GRAY));
    hoodArm =
        turretPost.append(
            new LoggedMechanismLigament2d(
                "HoodArm", ShooterConstants.hoodArmLengthMeters * 2.0, 0.0, 4, CYAN));
    flywheelIndicator =
        hoodArm.append(new LoggedMechanismLigament2d("Flywheel", 0.1, 0.0, 8, RED));

    // --- top-down view: turret yaw pointer + range limits ---
    // 90deg = up on canvas = robot forward; yaw sweeps left/right
    LoggedMechanismRoot2d turretRoot = topDown.getRoot("TurretRoot", 0.5, 0.5);
    turretPointer =
        turretRoot.append(new LoggedMechanismLigament2d("TurretPointer", 0.4, 90.0, 6, GREEN));
    rangeLeft =
        turretRoot.append(
            new LoggedMechanismLigament2d(
                "RangeLeft",
                0.3,
                90.0 + Units.radiansToDegrees(Turret.maxAngleRad),
                2,
                DARK_GRAY));
    rangeRight =
        turretRoot.append(
            new LoggedMechanismLigament2d(
                "RangeRight",
                0.3,
                90.0 + Units.radiansToDegrees(Turret.minAngleRad),
                2,
                DARK_GRAY));
  }

  /**
   * updates the mechanism visualization with current shooter state
   *
   * @param turretAngleRad current turret angle in radians
   * @param hoodAngleRad current hood angle in radians
   * @param flywheelVelocity current flywheel velocity in rad/s
   * @param flywheelAtGoal whether the flywheel is at its goal velocity
   */
  public void update(
      double turretAngleRad, double hoodAngleRad, double flywheelVelocity, boolean flywheelAtGoal) {
    // side view: hood arm angle relative to turret post (post points up at 90deg)
    hoodArm.setAngle(Units.radiansToDegrees(hoodAngleRad) - 90.0);

    // side view: flywheel indicator color + length
    flywheelIndicator.setColor(flywheelAtGoal ? GREEN : RED);
    double velocityFraction =
        Math.min(Math.abs(flywheelVelocity) / ShooterConstants.flywheelMaxVelocityRadsPerSec, 1.0);
    flywheelIndicator.setLength(0.05 + velocityFraction * 0.3);

    // Top-down view: turret yaw (90 = forward, positive = CCW = left)
    turretPointer.setAngle(90.0 + Units.radiansToDegrees(turretAngleRad));

    Logger.recordOutput("Shooter/SideView", sideView);
    Logger.recordOutput("Shooter/TurretTopDown", topDown);
  }
}
