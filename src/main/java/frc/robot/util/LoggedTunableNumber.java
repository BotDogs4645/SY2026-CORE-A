package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** gets value from dashboard in tuning mode, returns default if not or value not in dashboard */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey key on dashboard
   * @param defaultValue default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * set the default value of the number, the default value can only be set once
   *
   * @param defaultValue default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.tuningMode && !Constants.disableHAL) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  /**
   * get the current value, from dashboard if available and in tuning mode
   *
   * @return current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Constants.tuningMode && !Constants.disableHAL ? dashboardNumber.get() : defaultValue;
    }
  }

  /**
   * checks whether the number has changed since our last check
   *
   * @param id unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. recommended approach is to pass the result of "hashCode()"
   * @return true if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * runs action if any of the tunableNumbers have changed
   *
   * @param id unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. recommended approach is to pass the result of "hashCode()"
   * @param action callback to run when any of the tunable numbers have changed. parameters are the
   *     current values of the tunable numbers.
   * @param tunableNumbers all tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /** runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
