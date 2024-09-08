package frc.robot.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

public class Limelight {
    private NetworkTable table;
    private Pose3d pose;

    public Limelight(String name, int pipeline) {
        table = NetworkTableInstance.getDefault().getTable(name);
        table.getEntry("pipline").setNumber(pipeline);
        pose = null;
    }

    public Limelight(String name, int pipeline, Pose3d pose) {
        table = NetworkTableInstance.getDefault().getTable(name);
        table.getEntry("pipline").setNumber(pipeline);
        this.pose = pose;
    }

    public boolean hasTargets() {
        if (RobotBase.isReal() || pose == null) {
            return table.getEntry("tv").getNumber(0).intValue() == 1;
        }
        return false;
        
    }

    public double getTX() {
        if (RobotBase.isReal() || pose == null) {
            return -(double) table.getEntry("tx").getNumber(0);
        }
        return 0;
    }

    public double getTY() {
        if (RobotBase.isReal() || pose == null) {
            return (double) table.getEntry("ty").getNumber(0);
        }
        return 0;
    }

    public double getTA() {
        if (RobotBase.isReal() || pose == null) {
            return (double) table.getEntry("ta").getNumber(100);
        }
        return 0;
    }

    public double getLatency() {
        if (RobotBase.isReal()) {
            return (double) table.getEntry("cl").getNumber(0) + (double) table.getEntry("tl").getNumber(0);
        }
        return 0;
    }

    public PoseEstimate getPoseMT1() {
        double[] raw = table.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
        return new PoseEstimate(
            new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
            raw[6] * 1000,
            (int) raw[7],
            raw[9],
            raw[10],
            hasTargets()
        );
    }

    public PoseEstimate getPoseMT2(Rotation2d currentRotation, Rotation2d currentRotationRate) {
        if (Math.abs(currentRotationRate.getDegrees()) > 5) return getPoseMT1();
        table.getEntry("robot_orientation_set").setDoubleArray(new double[] {currentRotation.getDegrees(), 0, 0, 0, 0, 0});
        double[] raw = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
        return new PoseEstimate(
            new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
            raw[6] * 1000,
            (int) raw[7],
            raw[9],
            raw[10],
            hasTargets()
        );
    }

    public static record PoseEstimate(Pose2d pose, double latencySeconds, int tagCount, double averageDistance, double averageArea, boolean exists) {}
    public static record PieceEstimate(double tx, double ty, double area, double latencySeconds, boolean exists) {}
}
