package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.EncodedMotorController;

public class SwerveModule {
	private EncodedMotorController driveMotor;
	private EncodedMotorController angleMotor;
	private Translation2d translationFromCenter;

	public SwerveModule(
		EncodedMotorController driveMotor,
		EncodedMotorController angleMotor,
		Translation2d translationToCenter
	) {
		this.driveMotor = driveMotor;
		this.angleMotor = angleMotor;
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = optimizeTalon(
			initialTargetState,
			getModuleState().angle
		);
		setModuleVelocity(
			targetState.speedMetersPerSecond * 
            // Scale velocity by how far wheel is from target
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle.getRadians());
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER_METERS /
			2,
			new Rotation2d(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO)
		);
	}

	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity() * SwerveConstants.ANGLE_RATIO;
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle() /
			(2 * Math.PI) * 
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER_METERS *
			Math.PI,
			getModuleState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	public void setModuleAngle(double targetAngleRadians) {
		angleMotor.setAngle(targetAngleRadians / SwerveConstants.ANGLE_RATIO);
	}

	public void setModuleVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			targetVelocityMetersPerSecond * 2 /
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER_METERS)
		);
	}

    /**
	 * Minimize the change in heading the desired swerve module state would require
	 * by potentially reversing the direction the wheel spins. Customized from
	 * WPILib's version to include placing in appropriate scope for CTRE onboard
	 * control.
	 * 
	 * @see <a
	 *      href=https://www.chiefdelphi.com/t/swerve-modules-flip-180-degrees-periodically-conditionally/393059/3
	 *      >Chief Delphi Post Concerning The Issue</a>
	 * 
	 * @param desiredState The desired state.
	 * @param currentAngle The current module angle.
	 */
	private SwerveModuleState optimizeTalon(SwerveModuleState desiredState, Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
		double targetSpeed = desiredState.speedMetersPerSecond;
		return adjustTargetAngleAndSpeed(targetAngle, targetSpeed, currentAngle.getDegrees());
	}

	/**
	 * Adjusts the target angle and speed based on the current angle of the swerve
	 * module.
	 * If the difference between the target angle and current angle is greater than
	 * 90 degrees,
	 * the target speed is negated and the target angle is adjusted by 180 degrees.
	 *
	 * @param targetAngle  the desired angle for the swerve module to reach
	 * @param targetSpeed  the desired speed for the swerve module to reach
	 * @param currentAngle the current angle of the swerve module
	 * @return Optimized target module state
	 */
	private SwerveModuleState adjustTargetAngleAndSpeed(
			double targetAngle,
			double targetSpeed,
			double currentAngle) {
		double delta = targetAngle - currentAngle;
		if (Math.abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}
		return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * Places the given angle in the appropriate 0 to 360 degree scope based on the
	 * reference angle.
	 * 
	 * @param scopeReference the reference angle to place the new angle in scope of
	 * @param newAngle       the angle to place in the scope
	 * @return the new angle within the appropriate 0 to 360 degree scope
	 */
	private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double delta = newAngle - scopeReference;
		delta += 180; // shift range to [0, 360]
		delta %= 360; // normalize to [0, 360]
		if (delta < 0)
			delta += 360; // correct negative values
		delta -= 180; // shift range back to [-180, 180]
		return scopeReference + delta;
	}
}
