/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;

public class NavX {
	private AHRS ahrs;
	private double gyroZero;

	public NavX(edu.wpi.first.wpilibj.I2C.Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = 0;
	}

	public NavX(Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = 0;
	}

	public Rotation2d getAngleAsRotation2d() {
		return ahrs.getRotation2d();
	}

	/** Interval: [-infinity, infinity] @return Radians */
	public double getRawAngle() {
		return ahrs.getRotation2d().getRadians();
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getOffsetedAngle() {
		return getWrappedYaw() - getGyroZero();
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getWrappedYaw() {
		return -Math.toRadians(ahrs.getYaw());
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getWrappedPitch() {
		return -Math.toRadians(ahrs.getPitch());
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getWrappedRoll() {
		return -Math.toRadians(ahrs.getRoll());
	}

	public double getGyroZero() {
		return gyroZero;
	}

	public void zeroGyro() {
		gyroZero = getOffsetedAngle();
	}

	public void setGyroZero(double newZeroRadians) {
		gyroZero = newZeroRadians;
	}

	public void offsetGyroZero(double offsetRadians) {
		gyroZero -= offsetRadians;
	}
}
