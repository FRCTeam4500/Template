package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.EnumConstants.TalonModel;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonMotorController;
import frc.robot.hardware.interfaces.SwerveMotorController;

public class SwerveModule {
	private SwerveMotorController driveMotor;
	private SwerveMotorController angleMotor;
	private Translation2d translationFromCenter;

	public SwerveModule(
		SwerveMotorConfig driveConfig,
		SwerveMotorConfig angleConfig,
		Translation2d translationToCenter
	) {
		driveMotor = driveConfig.isNeo ? 
			new SparkMaxMotorController(driveConfig.canId, MotorType.kBrushless) :
			new TalonMotorController(driveConfig.canId, TalonModel.TalonFX);

		angleMotor = angleConfig.isNeo ? 
			new SparkMaxMotorController(angleConfig.canId, MotorType.kBrushless) :
			new TalonMotorController(angleConfig.canId, TalonModel.TalonFX);

		driveMotor.configureForSwerve(driveConfig);
		angleMotor.configureForSwerve(angleConfig);
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = SwerveModuleState.optimize(
			initialTargetState,
			getModuleState().angle
		);
		setModuleVelocity(
			targetState.speedMetersPerSecond  *// This is scales the velocity by how off the wheel is from the target angle.
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle.getRadians());
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER /
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
			SwerveConstants.WHEEL_DIAMETER *
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
			targetVelocityMetersPerSecond *
			2 /
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER)
		);
	}

	public static class SwerveMotorConfig {
		public boolean isDriveMotor;
		public boolean isNeo;
		public boolean invert;
		public int canId;
		public int currentLimit;
		public PIDConstants pid;
		public SwerveMotorConfig(
			int canId, boolean isDriveMotor, boolean invert, 
			boolean isNeo, int currentLimit, PIDConstants pid) {
			this.canId = canId;
			this.isDriveMotor = isDriveMotor;
			this.invert = invert;
			this.isNeo = isNeo;
			this.currentLimit = currentLimit;
			this.pid = pid;
		}
	}
}
