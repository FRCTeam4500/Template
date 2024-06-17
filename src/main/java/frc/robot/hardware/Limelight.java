package frc.robot.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable table;

    public Limelight(String name, int pipeline) {
        table = NetworkTableInstance.getDefault().getTable(name);
        table.getEntry("pipline").setNumber(pipeline);
    }

    public boolean hasTargets() {
        return table.getEntry("tv").getNumber(0).intValue() == 1;
    }

    public double getTX() {
        return -(double) table.getEntry("tx").getNumber(0);
    }

    public double getTY() {
        return (double) table.getEntry("ty").getNumber(0);
    }

    public double getTA() {
        return (double) table.getEntry("ta").getNumber(100);
    }

    public double getLatency() {
        return (double) table.getEntry("cl").getNumber(0) + (double) table.getEntry("tl").getNumber(0);
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
}
