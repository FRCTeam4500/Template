package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;

public interface EncodedMotorController {
	/** Units are radians/sec */
	public void setAngularVelocity(double targetAngularVelocity);

	/** Units are radians/sec */
	public double getAngularVelocity();

	/** Units are radians */
	public void setAngle(double targetAngle);

	/** Units are radians */
	public double getAngle();

	/** Units are percent */
	public void setOutput(double targetOutput);

	/** Units are percent */
	public double getOutput();

	/** Units are amps */
	public EncodedMotorController setCurrentLimit(int currentLimit);

	public EncodedMotorController setPID(PIDConstants pid);

	/** Units are radians */
	public EncodedMotorController setMinAngle(double minAngle);

	/** Units are radians */
	public EncodedMotorController setMaxAngle(double maxAngle);

	/** Units are percent */
	public EncodedMotorController setMinOutput(double minOutput);

	/** Units are percent */
	public EncodedMotorController setMaxOutput(double maxOutput);

	public EncodedMotorController setInversion(boolean shouldInvert);

	public EncodedMotorController setBrakeOnIdle(boolean shouldBreak);

	/** Units are radians */
	public EncodedMotorController setAngleTolerance(double tolerance);
}
