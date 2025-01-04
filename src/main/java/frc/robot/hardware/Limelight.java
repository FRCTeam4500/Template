package frc.robot.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/**
 * Wrapper class for limelights. An object of this class doesn't own the underlying hardware. In
 * other words, multiple objects can be made for one limelight, and they will all work. That is not
 * recommended though, since each object could change the pipeline.
 *
 * <pre>
 * //Example Usage
 * // Create a limelight, using the limelight named hehehe
 * // Set the pipeline to 0
 * Limelight limelight = new Limelight("limelight-hehehe", 0);
 *
 * double verticalOffset = limelight.getTY();
 * PoseEstimate estimate = limelight.getPoseMT1();
 * if (estimate.exists()) { // Only add the measurement if we have an estimate
 *     double timestamp = Timer.getFPGATimestamp() - estimate.latencySeconds();
 *     estimator.addVisionMeasurement(estimate.pose(), timestamp);
 * }
 * </pre>
 */
public class Limelight implements Loggable {
  private NetworkTable table;

  /**
   * Make a limelight with the given name and pipeline
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   * @param pipeline The pipeline to be used. These are configured in a web browser.
   */
  public Limelight(String name, int pipeline) {
    table = NetworkTableInstance.getDefault().getTable(name);
    table.getEntry("pipline").setInteger(pipeline);
  }

  /**
   * Make a limelight with the given name and pipeline 0
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   */
  public Limelight(String name) {
    this(name, 0);
  }

  /**
   * Make a gamepiece tracking limelight. This camera is automatically added to the {@link
   * GamePieceManager}
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   * @param pipeline The pipeline to be used. These are configured in a web browser.
   * @param pose The pose of this limelight relative to the bottom center of the robot. This pose is
   *     used to update the reading of gamepieces in sim
   */
  public Limelight(String name, int pipeline, Pose3d pose) {
    table = NetworkTableInstance.getDefault().getTable(name);
    table.getEntry("pipline").setInteger(pipeline);
    GamepieceManager.addCamera(name, pose);
  }

  /**
   * Make a gamepiece tracking limelight with the pipeline 0. This camera is automatically added to
   * the {@link GamePieceManager}
   *
   * @param name The name of the limelight. Should be "limelight-xxx"
   * @param pose The pose of this limelight relative to the bottom center of the robot. This pose is
   *     used to update the reading of gamepieces in sim
   */
  public Limelight(String name, Pose3d pose) {
    this(name, 0, pose);
  }

  /**
   * @return whether the limelight sees a target
   */
  public boolean hasTargets() {
    return table.getEntry("tv").getInteger(0) == 1;
  }

  /**
   * @return the horizontal offset from the target in counterclockwise positive degrees. Defaults to
   *     0 if no target is seen
   */
  public double getTX() {
    return -table.getEntry("tx").getDouble(0);
  }

  /**
   * @return the vertical offset from the target in degrees. Defaults to 0 if no target is seen
   */
  public double getTY() {
    return table.getEntry("ty").getDouble(0);
  }

  /**
   * @return the area taken up by the target in %. Defaults to 0 if no target is seen
   */
  public double getTA() {
    return table.getEntry("ta").getDouble(0);
  }

  /**
   * @return the total latency of all data from the limelight to the code
   */
  public double getLatency() {
    return table.getEntry("cl").getDouble(0) + table.getEntry("tl").getDouble(0);
  }

  /**
   * @return a {@link PoseEstimate} holding information about an estimated pose obtained using the
   *     megatag 1 algorithm.
   */
  public PoseEstimate getPoseMT1() {
    double[] raw = table.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
    return new PoseEstimate(
        new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
        raw[6] * 1000,
        (int) raw[7],
        raw[9],
        raw[10],
        hasTargets());
  }

  /**
   * @param currentRotation the current field relative rotation of the robot
   * @return a {@link PoseEstimate} holding information about an estimated pose obtained using the
   *     megatag 2 algorithm.
   */
  public PoseEstimate getPoseMT2(Rotation2d currentRotation) {
    table
        .getEntry("robot_orientation_set")
        .setDoubleArray(new double[] {currentRotation.getDegrees(), 0, 0, 0, 0, 0});
    NetworkTableInstance.getDefault().flush();
    double[] raw = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
    return new PoseEstimate(
        new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
        raw[6] * 1000,
        (int) raw[7],
        raw[9],
        raw[10],
        hasTargets());
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Has Targets", hasTargets());
    HoundLog.log(path, "Horizontal Angle", getTX());
    HoundLog.log(path, "Vertical Angle", getTY());
    HoundLog.log(path, "Area", getTA());
    HoundLog.log(path, "Latency", getLatency());
  }

  /** Holds an estimated position from a vison system. */
  public static record PoseEstimate(
      Pose2d pose,
      double latencySeconds,
      int tagCount,
      double averageDistance,
      double averageArea,
      boolean exists) {}
}
