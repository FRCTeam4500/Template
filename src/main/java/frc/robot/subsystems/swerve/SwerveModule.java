package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
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
		SwerveModuleState targetState = optimizeModuleState(
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
			new Rotation2d(angleMotor.getAngleRadians() * SwerveConstants.ANGLE_RATIO)
		);
	}

	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity() * SwerveConstants.ANGLE_RATIO;
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngleRadians() /
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
	private SwerveModuleState optimizeModuleState(SwerveModuleState desiredState, Rotation2d currentAngle) {
		// if (angleMotor.hasContinuousRotation()) {
		// 	return SwerveModuleState.optimize(desiredState, currentAngle);
		// }
		// double newTargetAngle = desiredState.angle.plus(
		// 	Rotation2d.fromDegrees(
		// 		MathUtil.inputModulus(
		// 			desiredState.angle.getDegrees() 
		// 			- currentAngle.getDegrees() + 180, 
		// 			0, 
		// 			360
		// 		)- 180
		// 	)
		// ).getDegrees();
		// double targetVelocity = desiredState.speedMetersPerSecond;
		// double delta = newTargetAngle - currentAngle.getDegrees();
		// if (Math.abs(delta) > 90) {
		// 	targetVelocity *= -1;
		// 	newTargetAngle = newTargetAngle - Math.signum(delta) * 180;
		// }
		// return new SwerveModuleState(
		// 	targetVelocity, 
		// 	Rotation2d.fromDegrees(newTargetAngle)
		// );
        return SwerveModuleState.optimize(desiredState, currentAngle);
	}
}