package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;

public interface EncodedMotorController {
	public void setAngularVelocity(double targetAngularVelocity);
	public double getAngularVelocity();
	public void setAngle(double targetAngleRadians);
	public double getAngleRadians();
	public void setOutput(double targetPercentOutput);
	public double getPercentOutput();
	public boolean hasContinuousRotation();
	public EncodedMotorController setCurrentLimit(int currentLimitAmps);
	public EncodedMotorController setPID(PIDConstants pid);
	public EncodedMotorController setMinAngle(double minAngleRadians);
	public EncodedMotorController setMaxAngle(double maxAngleRadians);
	public EncodedMotorController setMinOutput(double minPercentOutput);
	public EncodedMotorController setMaxOutput(double maxPercentOutput);
	public EncodedMotorController setInversion(boolean shouldInvert);
	public EncodedMotorController setBrakeOnIdle(boolean shouldBreak);
	public EncodedMotorController setAngleTolerance(double toleranceAngleRadians);
}