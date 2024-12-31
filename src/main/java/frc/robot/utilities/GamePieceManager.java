package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.logging.HoundLog;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Tracks the position of gamepieces during simulation, and updates game piece limelight readings
 */
public class GamePieceManager {
  private static HashMap<NetworkTable, Pose3d> cameras = new HashMap<>();
  private static Set<Translation2d> pieces = new HashSet<>();

  /** Resets the field to a match start state */
  public static void resetField() {
    pieces.clear();
    // TODO: Add starting translations of the pieces here!!
    // Ex: pieces.add(new Translation2d(1, 2));
    pieces.add(new Translation2d(2.9, 7));
    pieces.add(new Translation2d(2.9, 5.55));
    pieces.add(new Translation2d(2.9, 4.1));
    pieces.add(new Translation2d(8.3, 7.44));
    pieces.add(new Translation2d(8.3, 5.78));
    pieces.add(new Translation2d(8.3, 4.11));
    pieces.add(new Translation2d(8.3, 2.44));
    pieces.add(new Translation2d(8.3, 0.77));
    pieces.add(new Translation2d(13.67, 7));
    pieces.add(new Translation2d(13.67, 5.55));
    pieces.add(new Translation2d(13.67, 4.1));
    log();
  }

  /**
   * Adds a piece to the field
   *
   * @param translation The position of the piece
   */
  public static void addPiece(Translation2d translation) {
    pieces.add(translation);
    log();
  }

  /**
   * Removes a piece from the field
   *
   * @param translation The position of the piece
   */
  public static void removePiece(Translation2d translation) {
    pieces.remove(translation);
    log();
  }

  /**
   * Registers a game piece camera with the simulation. It's nt values will now be updated
   *
   * @param name The name of the camera. For example: "limelight-hehehe"
   * @param offset The position of the camera relative to the bottom center of the robot
   */
  public static void addCamera(String name, Pose3d offset) {
    cameras.put(NetworkTableInstance.getDefault().getTable(name), offset);
  }

  private static void log() {
    Translation2d[] array = new Translation2d[pieces.size()];
    int i = 0;
    for (Translation2d piece : pieces) {
      array[i] = piece;
      i++;
    }
    HoundLog.log("Pieces", array);
  }

  /**
   * Updates the network tables off all cameras registered from {@link #addCamera}. Note that calls
   * on a real robot will be silently ignorned, since this method can be very expensive!!
   *
   * @param robotPose The current position of the robot
   */
  public static void updateNT(Pose2d robotPose) {
    if (RobotBase.isReal()) {
      return;
    }
    for (Map.Entry<NetworkTable, Pose3d> cameraEntry : cameras.entrySet()) {
      Pose3d offset = cameraEntry.getValue();
      Pose3d camera =
          new Pose3d(
              robotPose.getX() + offset.getX(),
              robotPose.getY() + offset.getY(),
              offset.getZ(),
              new Rotation3d(
                  offset.getRotation().getX(),
                  offset.getRotation().getY(),
                  robotPose.getRotation().getRadians() + offset.getRotation().getZ()));
      boolean seenPiece = false;
      double upAngle = 0;
      double sideAngle = 0;
      for (Translation2d piece : pieces) {
        Pose3d poseVer = new Pose3d(new Pose2d(piece, new Rotation2d()));
        Pose3d thisPiece = poseVer.relativeTo(camera);
        double thisDist = thisPiece.getTranslation().getNorm();
        if (thisPiece.getX() < 0) {
          continue;
        }
        double thisUp = thisPiece.getZ();
        double thisSide = thisPiece.getY();
        double thisUpAngle = Math.toDegrees(Math.asin(thisUp / thisDist));
        double thisSideAngle = Math.toDegrees(Math.asin(thisSide / thisDist));
        if (seenPiece) {
          double thisCenterOffset = Math.hypot(thisSideAngle, thisUpAngle + 30);
          double seenCenterOffset = Math.hypot(sideAngle, upAngle + 30);
          if (thisCenterOffset < seenCenterOffset) {
            upAngle = thisUpAngle;
            sideAngle = thisSideAngle;
          }
        } else {
          if (Math.abs(thisUpAngle) < 25 && Math.abs(thisSideAngle) < 30) {
            seenPiece = true;
            upAngle = thisUpAngle;
            sideAngle = thisSideAngle;
          }
        }
      }
      NetworkTable table = cameraEntry.getKey();
      if (seenPiece) {
        table.getEntry("tv").setInteger(1);
        table.getEntry("tx").setNumber(-sideAngle);
        table.getEntry("ty").setNumber(upAngle);
      } else {
        table.getEntry("tv").setInteger(0);
        table.getEntry("tx").setNumber(0);
        table.getEntry("ty").setNumber(0);
      }
    }
  }
}
