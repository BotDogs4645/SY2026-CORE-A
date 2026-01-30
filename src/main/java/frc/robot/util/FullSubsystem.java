package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Extension of SubsystemBase that adds a periodicAfterScheduler() callback, called after the
 * command scheduler has run all periodic methods and commands. This ensures outputs are applied
 * after all commands have had a chance to update goals.
 */
public abstract class FullSubsystem extends SubsystemBase {
  private static final List<FullSubsystem> instances = new ArrayList<>();

  public FullSubsystem() {
    instances.add(this);
  }

  /** Called after the command scheduler runs each cycle. */
  public abstract void periodicAfterScheduler();

  /** Runs periodicAfterScheduler() on all FullSubsystem instances. */
  public static void runAllPeriodicAfterScheduler() {
    for (var instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
