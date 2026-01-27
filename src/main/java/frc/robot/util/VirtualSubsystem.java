package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/**
 * A virtual subsystem runs periodic code without being a command-based subsystem. Useful for
 * systems that need to run periodically but don't control hardware directly.
 */
public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  /** runs all virtual subsystem periodic methods */
  public static void periodicAll() {
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  /** override this method to run periodic code */
  public abstract void periodic();
}
