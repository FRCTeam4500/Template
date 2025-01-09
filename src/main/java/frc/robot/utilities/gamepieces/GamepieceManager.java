package frc.robot.utilities.gamepieces;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.logging.HoundLog;

public class GamepieceManager {
    private static HashSet<Gamepiece> pieces = new HashSet<>();
    private static HashMap<NetworkTable, Pose3d> cameras = new HashMap<>();
    private static Optional<Supplier<Pose2d>> robotPoseSupplier = Optional.empty();
    private static Optional<Gamepiece> heldPiece = Optional.of(new Gamepiece(new Translation2d()));
    private static Optional<Supplier<Transform3d>> heldTransformSupplier = Optional.of(() -> new Transform3d(0.15, 0, 0.5, new Rotation3d(0,-Math.PI / 3, 0))); 

    public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        robotPoseSupplier = Optional.of(supplier);
    }

    public static void addCamera(String name, Pose3d offset) {
        cameras.put(NetworkTableInstance.getDefault().getTable(name), offset);
    }

    public static void addPiece(Gamepiece piece) {
        pieces.add(piece);
    }

    public static void removePiece(Gamepiece piece) {
        pieces.remove(piece);
    }

    public static void resetField() {
        pieces.clear();
        // TODO: Add starting translations of the pieces here!!
        addPiece(new Gamepiece(new Translation2d(2.9, 7)));
        addPiece(new Gamepiece(new Translation2d(2.9, 5.55)));
        addPiece(new Gamepiece(new Translation2d(2.9, 4.1)));
        addPiece(new Gamepiece(new Translation2d(8.3, 7.44)));
        addPiece(new Gamepiece(new Translation2d(8.3, 5.78)));
        addPiece(new Gamepiece(new Translation2d(8.3, 4.11)));
        addPiece(new Gamepiece(new Translation2d(8.3, 2.44)));
        addPiece(new Gamepiece(new Translation2d(8.3, 0.77)));
        addPiece(new Gamepiece(new Translation2d(13.67, 7)));
        addPiece(new Gamepiece(new Translation2d(13.67, 5.55)));
        addPiece(new Gamepiece(new Translation2d(13.67, 4.1)));
    }

    public static void simulate() {
        Gamepiece[] toRemove = pieces.stream().filter(piece -> piece.shouldDelete()).toArray(Gamepiece[]::new);
        for (Gamepiece piece : toRemove) {
            removePiece(piece);
        }
        if (heldPiece.isPresent() && robotPoseSupplier.isPresent() && heldTransformSupplier.isPresent()) {
            heldPiece.get().setPose(new Pose3d(robotPoseSupplier.get().get()).transformBy(heldTransformSupplier.get().get()));
            pieces.add(heldPiece.get());
        }
        Pose3d[] pieceArray = pieces.stream().map(piece -> piece.getPose()).toArray(Pose3d[]::new);
        HoundLog.log("Pieces", pieceArray);
        if (RobotBase.isReal() || !robotPoseSupplier.isPresent()) {
            return;
        }
        Pose2d robotPose = robotPoseSupplier.get().get();
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
                for (Pose3d piece : pieceArray) {
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
    }
}
