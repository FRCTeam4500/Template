/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight {
	private NetworkTable table;

	public Limelight(String limelightName) {
		table = NetworkTableInstance.getDefault().getTable(limelightName);
		setPipeline(0);
	}

	public boolean hasValidTargets() {
		double value = getEntry("tv");
		if (value == 1) {
			return true;
		} else {
			return false;
		}
	}

	public Rotation2d getHorizontalOffsetFromCrosshair() {
		return Rotation2d.fromDegrees(-getEntry("tx"));
	}

	public Rotation2d getVerticalOffsetFromCrosshair() {
		return Rotation2d.fromDegrees(getEntry("ty"));
	}

	public double getTargetArea() {
		return getEntry("ta");
	}

	public Rotation2d getSkew() {
		double rawDegrees = getEntry("ts");
		double adjustedDegrees;
		if (Math.abs(rawDegrees) < 45) {
			adjustedDegrees = -rawDegrees;
		} else {
			adjustedDegrees = -(90 + rawDegrees);
		}
		return Rotation2d.fromDegrees(adjustedDegrees);
	}

	public void setPipeline(int index) {
		setEntry("pipeline", index);
	}

	private double getEntry(String key) {
		return table.getEntry(key).getDouble(0);
	}

	private void setEntry(String key, Number value) {
		table.getEntry(key).setNumber(value);
	}

	public Pose2d getRobotPoseToField() {
		double[] raw = table.getEntry("botpose").getDoubleArray(new double[6]);
		return new Pose2d(
			raw[0],
			raw[1],
			Rotation2d.fromDegrees(raw[5])
		);
	}

	public Pose2d getRobotPoseToAlliance(Alliance alliance) {
		double[] raw = new double[6];
		switch(alliance) {
			case Red:
				raw = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
                break;
			case Blue:
				raw = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
                break;
			default:
				break;
		}
		return new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5]));
	}

	public Pose2d getRobotPoseToTarget() {
		double[] raw = table
			.getEntry("botpose_targetspace")
			.getDoubleArray(new double[6]);
		return new Pose2d(
			raw[0],
			raw[1],
			Rotation2d.fromDegrees(raw[5])
		);
	}

	public Pose2d getTargetPoseToCamera() {
		double[] raw = table
			.getEntry("targetpose_cameraspace")
			.getDoubleArray(new double[6]);
		return new Pose2d(
			raw[0],
			raw[1],
			Rotation2d.fromDegrees(raw[5])
		);
	}

	public Pose2d getTargetPoseToRobot() {
		double[] raw = table
			.getEntry("targetpose_robotspace")
			.getDoubleArray(new double[6]);
		return new Pose2d(
			raw[0],
			raw[1],
			Rotation2d.fromDegrees(raw[5])
		);
	}

	public Pose2d getCameraPoseToTarget() {
		double[] raw = table
			.getEntry("camerapose_targetspace")
			.getDoubleArray(new double[6]);
		return new Pose2d(
			raw[0],
			raw[1],
			Rotation2d.fromDegrees(raw[5])
		);
	}

	public int getTargetTagId() {
		return (int) table.getEntry("tid").getInteger(0);
	}
}
