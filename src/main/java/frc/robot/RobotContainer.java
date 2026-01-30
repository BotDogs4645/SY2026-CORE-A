package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.util.Rumble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings({
    "unused",
    "FieldCanBeLocal"
  }) // Subsystem runs via CommandScheduler and communicates via callback
  private final Vision vision;

  private VisionIOQuestNav questNavIO;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SwerveConfig.FrontLeft),
                new ModuleIOTalonFX(SwerveConfig.FrontRight),
                new ModuleIOTalonFX(SwerveConfig.BackLeft),
                new ModuleIOTalonFX(SwerveConfig.BackRight));

        questNavIO = new VisionIOQuestNav();

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                questNavIO);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(SwerveConfig.FrontLeft),
                new ModuleIOSim(SwerveConfig.FrontRight),
                new ModuleIOSim(SwerveConfig.BackLeft),
                new ModuleIOSim(SwerveConfig.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Configure alerts (rumble feedback)
    configureAlerts();

    // Configure rumble demos for drive team testing
    //    configureRumbleDemos();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset robot pose to origin when X button is pressed
    //    driver
    //        .x()
    //        .onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d()),
    // drive).ignoringDisable(true));

    // Reset QuestNav pose to match current drive pose when back button is pressed
    //    driver
    //        .y()
    //        .onTrue(
    //            Commands.runOnce(
    //                    () -> {
    //                      if (questNavIO != null) {
    //                        questNavIO.setPose(new Pose3d(drive.getPose()));
    //                      }
    //                    })
    //                .ignoringDisable(true));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Pose2d newPose =
                          new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero);
                      drive.setPose(newPose);
                      if (questNavIO != null) {
                        questNavIO.setPose(new Pose3d(newPose));
                      }
                    },
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** configures endgame alerts and other feedback triggers */
  private void configureAlerts() {
    // endgame alert #1 - single pulse at 30s
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Constants.ENDGAME_ALERT_1_TIME)
        .onTrue(
            Commands.parallel(
                    controllerRumbleCommand().withTimeout(0.5),
                    Commands.runOnce(() -> Leds.getInstance().endgameAlert = true))
                .andThen(Commands.waitSeconds(1.0))
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));

    // endgame alert #2 - triple pulse at 15s
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Constants.ENDGAME_ALERT_2_TIME)
        .onTrue(
            Commands.parallel(
                    controllerRumbleCommand()
                        .withTimeout(0.2)
                        .andThen(Commands.waitSeconds(0.1))
                        .repeatedly()
                        .withTimeout(0.9),
                    Commands.runOnce(() -> Leds.getInstance().endgameAlert = true))
                .andThen(Commands.waitSeconds(1.5))
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
  }

  /**
   * creates a command that rumbles the driver controller
   *
   * @return command that rumbles until interrupted
   */
  private Command controllerRumbleCommand() {
    return Rumble.rumble(driver.getHID(), 1.0);
  }

  /**
   * returns a supplier for creating rumble commands for injection into other commands
   *
   * @return supplier that creates rumble commands
   */
  public Supplier<Command> getRumbleSupplier() {
    return this::controllerRumbleCommand;
  }

  /**
   * configures rumble demo bindings for drive team to test different feedback types. uses D-pad and
   * Y button. u better not forget to remove this method before competition.
   */
  private void configureRumbleDemos() {
    GenericHID hid = driver.getHID();

    // full intensity both motors
    driver.povUp().whileTrue(Rumble.rumble(hid, 1.0));

    // 40% intensity both motors
    driver.povDown().whileTrue(Rumble.rumble(hid, 0.4));

    // full intensity left motor only
    driver.povLeft().whileTrue(Rumble.rumbleLeft(hid, 1.0));

    // full intensity right motor only
    driver.povRight().whileTrue(Rumble.rumbleRight(hid, 1.0));

    // triple pulse pattern
    driver.y().onTrue(Rumble.rumblePattern(hid, 1.0, 0.25, 0.15, 3));

    // left right left right
    driver
        .leftBumper()
        .and(driver.rightBumper())
        .whileTrue(Rumble.rumbleAlternating(hid, 1.0, 0.30, 10));
  }
}
