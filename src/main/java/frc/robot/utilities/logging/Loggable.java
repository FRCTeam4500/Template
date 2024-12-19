package frc.robot.utilities.logging;

/** A {@link FunctionalInterface} for an object that can be logged */
@FunctionalInterface
public interface Loggable {
  /**
   * Logs this object, and calls any loggables held by this object
   *
   * @param name The file path of the logged data
   * @implNote When logging, the passed in name, as well as a "/" should be added before the key
   *     <pre>
   * // Example log body
   * public void log(String name) {
   *   HoundLog.log(name + "/MyDouble", 5);
   *   HoundLog.log(name + "/MyString", "hi");
   * }
   * </pre>
   */
  public void log(String name);
}
