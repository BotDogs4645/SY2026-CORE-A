package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;

/**
 * Builds SwerveModuleConstants from DriveConstants (robot-specific) and WheelConstants
 * (wheel-specific). This allows any combination of robot and wheel configurations.
 */
public class SwerveConfig {
  // Closed-loop output types
  private static final SwerveModuleConstants.ClosedLoopOutputType steerClosedLoopOutput =
      SwerveModuleConstants.ClosedLoopOutputType.Voltage;
  private static final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput =
      SwerveModuleConstants.ClosedLoopOutputType.Voltage;

  // Motor types
  private static final DriveMotorArrangement driveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement steerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;
  private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // Initial motor configs
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

  // Simulation constants
  private static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.025);
  private static final Voltage steerFrictionVoltage = Volts.of(0.2);
  private static final Voltage driveFrictionVoltage = Volts.of(0.2);

  // Factory for creating module constants
  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(DriveConstants.driveGearRatio)
              .withSteerMotorGearRatio(DriveConstants.steerGearRatio)
              .withCouplingGearRatio(DriveConstants.coupleRatio)
              .withWheelRadius(WheelConstants.wheelRadius)
              .withSteerMotorGains(WheelConstants.steerGains)
              .withDriveMotorGains(WheelConstants.driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSlipCurrent(WheelConstants.slipCurrent)
              .withSpeedAt12Volts(WheelConstants.speedAt12Volts)
              .withDriveMotorType(driveMotorType)
              .withSteerMotorType(steerMotorType)
              .withFeedbackSource(steerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(steerInertia)
              .withDriveInertia(driveInertia)
              .withSteerFrictionVoltage(steerFrictionVoltage)
              .withDriveFrictionVoltage(driveFrictionVoltage);

  // Create module constants from config index
  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModule(int index) {
    ModuleConfig config = DriveConstants.moduleConfigs[index];
    boolean invertDrive =
        (index == 0 || index == 2) ? DriveConstants.invertLeftSide : DriveConstants.invertRightSide;

    return constantCreator.createModuleConstants(
        config.steerMotorId(),
        config.driveMotorId(),
        config.encoderId(),
        config.encoderOffset(),
        Meters.of(DriveConstants.moduleTranslations[index].getX()),
        Meters.of(DriveConstants.moduleTranslations[index].getY()),
        invertDrive,
        config.steerInverted(),
        config.encoderInverted());
  }

  // Pre-built module constants
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft = createModule(0);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight = createModule(1);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft = createModule(2);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight = createModule(3);
}
