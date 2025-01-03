package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utilities.logging.HoundLog;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Tracks the position of gamepieces during simulation, and updates game piece limelight readings
 */
public class GamePieceManager {
  private static HashMap<NetworkTable, Pose3d> cameras = new HashMap<>();
  private static Set<Pose3d> pieces = new HashSet<>();

  /** Resets the field to a match start state */
  public static void resetField() {
    pieces.clear();
    // TODO: Add starting translations of the pieces here!!
    pieces.add(new Pose3d(2.9, 7, 0, new Rotation3d()));
    pieces.add(new Pose3d(2.9, 7, 0, new Rotation3d()));
    pieces.add(new Pose3d(2.9, 5.55, 0, new Rotation3d()));
    pieces.add(new Pose3d(2.9, 4.1, 0, new Rotation3d()));
    pieces.add(new Pose3d(8.3, 7.44, 0, new Rotation3d()));
    pieces.add(new Pose3d(8.3, 5.78, 0, new Rotation3d()));
    pieces.add(new Pose3d(8.3, 4.11, 0, new Rotation3d()));
    pieces.add(new Pose3d(8.3, 2.44, 0, new Rotation3d()));
    pieces.add(new Pose3d(8.3, 0.77, 0, new Rotation3d()));
    pieces.add(new Pose3d(13.67, 7, 0, new Rotation3d()));
    pieces.add(new Pose3d(13.67, 5.55, 0, new Rotation3d()));
    pieces.add(new Pose3d(13.67, 4.1, 0, new Rotation3d()));
    log();
  }

  /**
   * Adds a piece to the field
   *
   * @param translation The position of the piece
   */
  public static void addPiece(Translation2d translation) {
    pieces.add(new Pose3d(translation.getX(), translation.getY(), 0, new Rotation3d()));
    log();
  }

  /**
   * Removes a piece from the field
   *
   * @param translation The position of the piece
   */
  public static void removePiece(Translation2d translation) {
    pieces.remove(new Pose3d(translation.getX(), translation.getY(), 0, new Rotation3d()));
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
    Pose3d[] array = new Pose3d[pieces.size()];
    int i = 0;
    for (Pose3d piece : pieces) {
      array[i] = piece;
      i++;
    }
    HoundLog.log("Pieces", array);
  }

  public static Command animatePiece(Pose3d start, Pose3d end, double duration) {
    return Commands.defer(
      () -> {
        Timer timer = new Timer();
        timer.start();
        return Commands.run(() -> {
          HoundLog.log(
            "Animated Piece", 
            new Pose3d[] {
              start.interpolate(end, timer.get() / duration)
            });
        })
        .until(() -> timer.hasElapsed(duration))
        .finallyDo(() -> HoundLog.log("Animated Piece", new Pose3d[] {}));
      },
      Set.of()
    );
  }

  /**
   * A command that updates the network tables of the game piece cameras added via {@link #addCamera}.
   * @param robotPoseSupplier A functions that returns the robot's current position
   * @return A command that updates the camera's NT values. If this isn't a simulation, returns a blank command.
   */
  public static Command updateNT(Supplier<Pose2d> robotPoseSupplier) {
    if (RobotBase.isReal()) {
      return Commands.idle().withName("Fake Gamepiece NT Command");
    }
    return Commands.run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();
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
        for (Pose3d piece : pieces) {
          Pose3d thisPiece = piece.relativeTo(camera);
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
    }).withName("Gamepiece NT Command");
  }
}
