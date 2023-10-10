package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.hardware.interfaces.SwerveMotorController;

public class SparkMaxMotorController extends CANSparkMax implements SwerveMotorController {

	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
	}

	public double getAngle() {
		return getEncoder().getPosition() * 2 * Math.PI;
	}

	public void setAngle(double position) {
		getPIDController()
			.setReference(position / (2 * Math.PI), ControlType.kPosition);
	}

	public void setOutput(double output) {
		set(output);
	}

	public double getOutput() {
		return get();
	}

	public double getAngularVelocity() {
		return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
	}

	/**
	 * set velocity, in rad/s
	 * @param velocity angular velocity, in rad/s
	 */
	public void setAngularVelocity(double velocity) {
		getPIDController()
			.setReference(
				Units.radiansPerSecondToRotationsPerMinute(velocity),
				ControlType.kVelocity
			);
	}

	public void configureForSwerve(boolean isInverted, int currentLimit, PIDConstants pid, boolean isDriveMotor) {
		setInverted(isInverted);
        getPIDController().setP(pid.kP);
        getPIDController().setI(pid.kI);
        getPIDController().setD(pid.kD);
        getPIDController().setFF(0);
        setSmartCurrentLimit(currentLimit);
	}
}
