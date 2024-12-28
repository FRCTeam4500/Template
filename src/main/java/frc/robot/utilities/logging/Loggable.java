package frc.robot.utilities.logging;

/** A {@link FunctionalInterface} for an object that can be logged */
@FunctionalInterface
public interface Loggable {
  /**
   * Logs this object, and calls any loggables held by this object
   *
   * <pre>
   * // Example log body
   * public void log(String path) {
   *   HoundLog.log(path, "MyDouble", 5);
   *   HoundLog.log(path, "MyString", "hi");
   * }
   * </pre>
   *
   * @param path The file path of the logged data
   */
  public void log(String path);
}
