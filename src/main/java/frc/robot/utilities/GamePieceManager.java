package frc.robot.utilities;

import java.util.Set;
import java.util.HashSet;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;

public class GamePieceManager {

    private static Set<Translation2d> pieces = new HashSet<>();
    public static void resetField() {
        pieces.clear();
        addPiece(new Translation2d(2.9, 7));
        addPiece(new Translation2d(2.9, 5.55));
        addPiece(new Translation2d(2.9, 4.1));
        addPiece(new Translation2d(8.3, 7.44));
        addPiece(new Translation2d(8.3, 5.78));
        addPiece(new Translation2d(8.3, 4.11));
        addPiece(new Translation2d(8.3, 2.44));
        addPiece(new Translation2d(8.3, 0.77));
        addPiece(new Translation2d(13.67, 7));
        addPiece(new Translation2d(13.67, 5.55));
        addPiece(new Translation2d(13.67, 4.1));
        log();
    }

    public static void addPiece(Translation2d translation) {
        pieces.add(translation);
        log();
    }

    public static void removePiece(Translation2d translation) {
        pieces.remove(translation);
        log();
    }

    public static boolean seesPiece(Pose2d robotPose, Pose3d cameraRelativePose) {
        Translation2d[] piecesRel = getRelativeToRobot(robotPose, cameraRelativePose);
        return false;
    }

    private static void log() {
        Translation2d[] array = new Translation2d[pieces.size()];
        int i = 0;
        for (Translation2d piece : pieces) {
            array[i] = piece;
            i++;
        }
        DogLog.log("Pieces", array);
    }

    public static void updateLimelightNT(
        NetworkTable table, 
        Pose2d robotPose, 
        Pose3d cameraOffsets
    ) {
        Rotation3d cameraRotation3d = new Rotation3d(
            cameraOffsets.getRotation().getX(), 
            cameraOffsets.getRotation().getY(), 
            robotPose.getRotation().getRadians() + cameraOffsets.getRotation().toRotation2d().getRadians()
        );
        Translation3d cameraTranslation3d = new Translation3d(
            robotPose.getX() + cameraOffsets.getX(), 
            robotPose.getY() + cameraOffsets.getY(), 
            cameraOffsets.getZ()
        );
        Pose3d camera = new Pose3d(cameraTranslation3d, cameraRotation3d);
        Pose3d seenPiece = null;
        for (Translation2d piece : pieces) {
            Pose3d poseVer = new Pose3d(new Pose2d(piece, new Rotation2d()));
            Pose3d thisPiece = poseVer.relativeTo(camera);
            double distance = thisPiece.getTranslation().getNorm();
            if (seenPiece == null) {
                double up = Math.abs(thisPiece.getZ());
                double sideways = Math.abs(thisPiece.getY());
                double upAngle = Math.toDegrees(Math.asin(up/distance));
                double sideAngle = Math.toDegrees(Math.asin(sideways/distance));
                if (upAngle < 25 && sideAngle < 30) {
                    seenPiece = thisPiece;
                } 
            } else {
                double thisDistance = thisPiece.getTranslation().toTranslation2d().getNorm();
                double seenDistance = seenPiece.getTranslation().toTranslation2d().getNorm();
                if (thisDistance < seenDistance) {
                    seenPiece = thisPiece;
                }
            }
        }
        if (seenPiece == null) {
            table.getEntry("tv").setInteger(0);
        } else {
            double distance = seenPiece.getTranslation().getNorm();
            double up = seenPiece.getZ();
            double sideways = seenPiece.getY();
            double upAngle = Math.toDegrees(Math.asin(up/distance));
            double sideAngle = Math.toDegrees(Math.asin(sideways/distance));
            table.getEntry("tv").setInteger(1);
            table.getEntry("tx").setNumber(-sideAngle);
            table.getEntry("ty").setNumber(upAngle);
            table.getEntry("ta").setNumber(0); //TODO: figure this out
        }
       
    }

    private static Translation2d[] getRelativeToRobot(Pose2d robotPose, Pose3d cameraRelativePose) {
        Rotation3d cameraRotation3d = new Rotation3d(
            cameraRelativePose.getRotation().getX(), 
            cameraRelativePose.getRotation().getY(), 
            robotPose.getRotation().getRadians() + cameraRelativePose.getRotation().toRotation2d().getRadians()
        );
        Translation3d cameraTranslation3d = new Translation3d(
            robotPose.getX() + cameraRelativePose.getX(), 
            robotPose.getY() + cameraRelativePose.getY(), 
            cameraRelativePose.getZ()
        );
        Pose3d camera = new Pose3d(cameraTranslation3d, cameraRotation3d);
        Translation2d[] piecesRel = new Translation2d[pieces.size()];
        int i = 0;
        for (Translation2d piece : pieces) {
            Pose3d poseVer = new Pose3d(new Pose2d(piece, new Rotation2d()));
            piecesRel[i] = poseVer.relativeTo(camera).getTranslation().toTranslation2d();
            i++;
        }
        return piecesRel;
    }
}
