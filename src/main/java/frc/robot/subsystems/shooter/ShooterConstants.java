package frc.robot.subsystems.shooter;

public class ShooterConstants {
  // TODO: CAN IDs
  public static final int turretMotorId = 20;
  public static final int hoodMotorId = 21;
  public static final int flywheelMotorId = 22;

  // TODO: gear ratios
  public static final double turretGearRatio = 16.5;
  public static final double hoodGearRatio = 1.0;
  public static final double flywheelGearRatio = 1.0;

  // TODO: physical offsets
  public static final double turretXOffsetMeters = 0.0;
  public static final double turretZOffsetMeters = 0.5;

  // shooter wheel
  public static final double shooterWheelRadiusMeters = 0.0508; // 2 inches

  // visualization constants
  public static final double hoodArmLengthMeters = 0.3;
  public static final double flywheelMaxVelocityRadsPerSec = 600.0;
  public static final double shotFlightDurationSecs = 1.0;
  public static final int shotTracerPointCount = 20;
}
