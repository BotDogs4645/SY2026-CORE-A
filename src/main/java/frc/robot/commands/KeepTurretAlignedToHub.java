// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KeepTurretAlignedToHub extends Command {

  private Turret turret;
  private Drive drive;

  /** Creates a new KeepTurretAlignedToHub. */
  public KeepTurretAlignedToHub(Turret turret, Drive drive) {
    this.turret = turret;
    this.drive = drive;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d drivePose = drive.getPose();
    Translation2d hubLocation = new Translation2d(4.634, 4.003);
    Translation2d vectorToHub = hubLocation.minus(drivePose.getTranslation());

    Logger.recordOutput("Turret/vectorToHub", vectorToHub);

    Rotation2d rotationToHub = new Rotation2d(vectorToHub.getX(), vectorToHub.getY());
    Logger.recordOutput("Turret/rotationToHub", rotationToHub.getDegrees());
    turret.setControl(
        new PositionDutyCycle(Angle.ofBaseUnits(rotationToHub.getRotations(), Rotations)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setControl(new CoastOut());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
