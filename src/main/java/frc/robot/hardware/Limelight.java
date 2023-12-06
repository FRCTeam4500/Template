/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import java.util.Optional;

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
		return getEntry("tv") == 1;
	}

	public Optional<Rotation2d> getHorizontalOffsetFromCrosshair() {
		if (!hasValidTargets()) return Optional.empty();
		return Optional.of(
			Rotation2d.fromDegrees(-getEntry("tx"))
		);
	}

	public Optional<Rotation2d> getVerticalOffsetFromCrosshair() {
		if (!hasValidTargets()) return Optional.empty();
		return Optional.of(
			Rotation2d.fromDegrees(getEntry("ty"))
		);
	}

	public Optional<Double> getTargetArea() {
		if (!hasValidTargets()) return Optional.empty();
		return Optional.of( 
			getEntry("ta")
		);
	}

	public Optional<Rotation2d> getSkew() {
		if (!hasValidTargets()) return Optional.empty();
		double rawDegrees = getEntry("ts");
		double adjustedDegrees;
		if (Math.abs(rawDegrees) < 45) {
			adjustedDegrees = -rawDegrees;
		} else {
			adjustedDegrees = -(90 + rawDegrees);
		}
		return Optional.of(
			Rotation2d.fromDegrees(adjustedDegrees)
		);
	}

	public void setPipeline(int index) {
		setEntry("pipeline", index);
	}

	private double getEntry(String key) {
		return table.getEntry(key).getDouble(0);
	}

	private double[] getEntryArray(String key) {
		return table.getEntry(key).getDoubleArray(new double[6]);
	}

	private void setEntry(String key, Number value) {
		table.getEntry(key).setNumber(value);
	}

	public Optional<Pose2d> getRobotPoseToField() {
		if (!hasValidTargets()) return Optional.empty();
		double[] raw = getEntryArray("botpose");
		return Optional.of(new Pose2d(
			raw[0], raw[1], Rotation2d.fromDegrees(raw[5])
		));
	}

	public Optional<Pose2d> getRobotPoseToAlliance(Alliance alliance) {
		if (!hasValidTargets()) return Optional.empty();
		double[] raw = new double[6];
		switch(alliance) {
			case Red:
				raw = getEntryArray("botpose_wpired");
                break;
			case Blue:
				raw = getEntryArray("botpose_wpiblue");
                break;
			default:
				break;
		}
		return Optional.of(new Pose2d(
			raw[0], raw[1], Rotation2d.fromDegrees(raw[5])
		));
	}

	public Optional<Pose2d> getRobotPoseToTarget() {
		if (!hasValidTargets()) return Optional.empty();
		double[] raw = getEntryArray("botpose_targetspace");
		return Optional.of(new Pose2d(
			raw[0], raw[1], Rotation2d.fromDegrees(raw[5])
		));
	}

	public Optional<Pose2d> getTargetPoseToCamera() {
		if (!hasValidTargets()) return Optional.empty();
		double[] raw = getEntryArray("targetpose_cameraspace");
		return Optional.of(new Pose2d(
			raw[0], raw[1], Rotation2d.fromDegrees(raw[5])
		));
	}

	public Optional<Pose2d> getTargetPoseToRobot() {
		if (!hasValidTargets()) return Optional.empty();
		double[] raw = getEntryArray("targetpose_robotspace");
		return Optional.of(new Pose2d(
			raw[0], raw[1], Rotation2d.fromDegrees(raw[5])
		));
	}

	public Optional<Pose2d> getCameraPoseToTarget() {
		if (!hasValidTargets()) return Optional.empty();
		double[] raw = getEntryArray("camerapose_targetspace");
		return Optional.of(new Pose2d(
			raw[0], raw[1], Rotation2d.fromDegrees(raw[5])
		));
	}

	public Optional<Integer> getTargetTagId() {
		if (!hasValidTargets()) return Optional.empty();
		return Optional.of(
			(int) getEntry("tid")
		);
	}
}
