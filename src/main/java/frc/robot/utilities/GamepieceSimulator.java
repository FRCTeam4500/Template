package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utilities.logging.HoundLog;
import java.util.ConcurrentModificationException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Tracks the position of gamepieces during simulation, and updates game piece limelight readings
 */
public class GamepieceSimulator {
  private static HashMap<NetworkTable, Pose3d> cameras = new HashMap<>();
  private static Set<Translation2d> pieces = new HashSet<>();
  private static Set<AnimatedGamepiece> animatedPieces = new HashSet<>();
  private static Supplier<Pose2d> robotPoseSupplier;

  static {
    if (RobotBase.isSimulation()) {
      animatePieces().schedule();
      updateNT().schedule();
    }
  }

  /** Resets the field to a match start state */
  public static void resetField() {
    animatedPieces.clear();
    pieces.clear();
    // TODO: Add starting translations of the pieces here!!
    pieces.add(new Translation2d(2.9, 7));
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

  public static void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
    robotPoseSupplier = poseSupplier;
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

  public static void animatePiece(Translation2d start, Pose3d end, double duration) {
    animatedPieces.add(
        new AnimatedGamepiece(
            new Pose3d(new Translation3d(start), new Rotation3d()), end, duration));
    removePiece(start);
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
    Pose3d[] array = new Pose3d[pieces.size()];
    int i = 0;
    for (Translation2d piece : pieces) {
      array[i] = new Pose3d(new Translation3d(piece), new Rotation3d());
      i++;
    }
    HoundLog.log("Pieces", array);
  }

  /**
   * @return A command that animates any pieces added by {@link #animatePiece}
   */
  private static Command animatePieces() {
    return Commands.run(
            () -> {
              try {
                for (AnimatedGamepiece piece : animatedPieces) {
                  if (piece.done()) {
                    animatedPieces.remove(piece);
                  }
                }
                Pose3d[] array = new Pose3d[animatedPieces.size()];
                int i = 0;
                for (AnimatedGamepiece piece : animatedPieces) {
                  array[i] = piece.getPose();
                  i++;
                }
                HoundLog.log("Animated Pieces", array);
              } catch (ConcurrentModificationException e) {
                // We dont have to do anything, stuff will be fixed next loop
              }
            })
        .ignoringDisable(true)
        .withName("Animating Pieces");
  }

  /**
   * A command that updates the network tables of the game piece cameras added via {@link
   * #addCamera}.
   *
   * @return A command that updates the camera's NT values. If this isn't a simulation, returns a
   *     blank command.
   */
  private static Command updateNT() {
    if (RobotBase.isReal()) {
      return Commands.idle().withName("Fake Gamepiece NT Command");
    }
    return Commands.run(
            () -> {
              if (robotPoseSupplier == null) {
                return;
              }
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
                for (Translation2d piece : pieces) {
                  Pose3d poseVer = new Pose3d(new Translation3d(piece), new Rotation3d());
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
            })
        .ignoringDisable(true)
        .withName("Gamepiece NT Command");
  }

  /** Holds info about a game piece that is animated */
  private static class AnimatedGamepiece {
    private Pose3d startPose;
    private Pose3d endPose;
    private double startTime;
    private double endTime;

    public AnimatedGamepiece(Pose3d start, Pose3d end, double duration) {
      startPose = start;
      endPose = end;
      startTime = Timer.getFPGATimestamp();
      endTime = startTime + duration;
    }

    public Pose3d getPose() {
      return startPose.interpolate(
          endPose, (Timer.getFPGATimestamp() - startTime) / (endTime - startTime));
    }

    public boolean done() {
      return Timer.getFPGATimestamp() > endTime;
    }

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof AnimatedGamepiece piece) {
        return piece.startPose.equals(this.startPose)
            && piece.endPose.equals(this.endPose)
            && piece.startTime == this.startTime
            && piece.endTime == this.endTime;
      } else {
        return false;
      }
    }
  }
}
