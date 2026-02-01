package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.subsystems.shooter.AutoShotCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.util.BallVisualizer;
import frc.robot.util.Rumble;
import frc.robot.util.ShooterMechanism;
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

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  private final ShooterMechanism shooterMechanism = new ShooterMechanism();
  private final AutoShotCalculator shotCalculator = new AutoShotCalculator();
  private AutoShotCalculator.ShotSolution latestSolution = AutoShotCalculator.ShotSolution.none();

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

        turret = new Turret(new TurretIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        flywheel = new Flywheel(new FlywheelIOTalonFX());

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

        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
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

        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
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

    // wire up visualizers
    BallVisualizer.setRobotPoseSupplier(drive::getPose);
    BallVisualizer.setShooterStateSuppliers(turret::getPosition, hood::getPosition);
    BallVisualizer.setBallCount(8);

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

    // turret holds position by default, hood holds, flywheel coasts
    turret.setDefaultCommand(turret.holdCommand());
    hood.setDefaultCommand(hood.holdCommand());
    flywheel.setDefaultCommand(flywheel.stopCommand().andThen(Commands.idle(flywheel)));

    // zero turret and hood when start button is pressed
    driver
        .start()
        .onTrue(Commands.parallel(turret.zeroCommand(), hood.zeroCommand()).ignoringDisable(true));

    // test turret to 0 degrees on left bumper
    operator.leftBumper().whileTrue(turret.runFixedCommand(0.0));

    // test flywheel spin-up on right bumper
    operator.rightBumper().whileTrue(flywheel.runFixedCommand(500.0));

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

    // shooter controls
    Trigger leftTriggerPressed = driver.leftTrigger();

    // left trigger: auto-aim turret + hood using shot calculator
    leftTriggerPressed
        .and(this::isOnCorrectAllianceSide)
        .whileTrue(
            Commands.run(
                    () -> {
                      Translation3d target = getHubTarget();
                      latestSolution =
                          shotCalculator.calculate(
                              drive.getPose(), drive.getChassisSpeeds(), target);
                      if (latestSolution.isSolutionFound()) {
                        turret.setGoal(latestSolution.turretAngleRad());
                        hood.setGoal(latestSolution.hoodAngleRad(), 0.0);
                      }
                    },
                    turret,
                    hood)
                .withName("AutoAim"));

    // aim-locked rumble (turret + hood both at goal with valid solution)
    Trigger aimLockedWhileAiming =
        leftTriggerPressed
            .and(this::isOnCorrectAllianceSide)
            .and(turret::atGoal)
            .and(hood::atGoal)
            .and(() -> latestSolution.isSolutionFound())
            .debounce(0.2, Debouncer.DebounceType.kBoth);

    aimLockedWhileAiming.onTrue(Rumble.rumblePulse(driver.getHID(), 1.0, 0.15));
    aimLockedWhileAiming.onFalse(
        Rumble.rumblePulse(driver.getHID(), 0.8, 0.5).onlyIf(leftTriggerPressed));

    // right trigger: fire (flywheel at calculated speed, or fallback)
    driver
        .rightTrigger()
        .and(() -> BallVisualizer.getBallCount() > 0)
        .whileTrue(
            Commands.run(
                    () -> {
                      double speed =
                          latestSolution.isSolutionFound()
                              ? latestSolution.flywheelVelocityRadPerSec()
                              : 500.0; // fallback if no solution
                      flywheel.setGoal(speed);
                    },
                    flywheel)
                .withName("Fire"));

    // wire shot detection to ball visualizer
    flywheel.shotDetectedTrigger().onTrue(BallVisualizer.shoot());
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

  /** returns true if the robot is on its own alliance side of the field */
  private boolean isOnCorrectAllianceSide() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    double robotX = drive.getPose().getX();
    double centerX = FieldConstants.fieldLength / 2.0;
    return isRed ? robotX > centerX : robotX < centerX;
  }

  /** returns the hub target position for the current alliance */
  private Translation3d getHubTarget() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    return isRed ? FieldConstants.Hub.oppTopCenterPoint : FieldConstants.Hub.topCenterPoint;
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
  public Pose2d getDrivePose() {
    return drive.getPose();
  }

  public ShooterMechanism getShooterMechanism() {
    return shooterMechanism;
  }

  public Turret getTurret() {
    return turret;
  }

  public Hood getHood() {
    return hood;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

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
