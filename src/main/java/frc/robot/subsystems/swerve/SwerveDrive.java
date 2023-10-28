package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utilities.Loggable;

public class SwerveDrive extends SubsystemBase implements Loggable {
	private static SwerveDrive instance;
	private NavX gyro;	
	private Vision vision;
	private SwerveModule[] modules;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveDrivePoseEstimator poseEstimator;

	private SwerveDrive() {
		modules = new SwerveModule[] {
			new SwerveModule(
				SwerveConstants.FRONT_LEFT_DRIVE_CONFIG, 
				SwerveConstants.FRONT_LEFT_ANGLE_CONFIG, 
				SwerveConstants.FRONT_LEFT_MODULE_TRANSLATION
			),
			new SwerveModule(
				SwerveConstants.FRONT_RIGHT_DRIVE_CONFIG, 
				SwerveConstants.FRONT_RIGHT_ANGLE_CONFIG, 
				SwerveConstants.FRONT_RIGHT_MODULE_TRANSLATION
			),
			new SwerveModule(
				SwerveConstants.BACK_LEFT_DRIVE_CONFIG, 
				SwerveConstants.BACK_LEFT_ANGLE_CONFIG, 
				SwerveConstants.BACK_LEFT_MODULE_TRANSLATION
			),
			new SwerveModule(
				SwerveConstants.BACK_RIGHT_DRIVE_CONFIG, 
				SwerveConstants.BACK_RIGHT_ANGLE_CONFIG, 
				SwerveConstants.BACK_RIGHT_MODULE_TRANSLATION
			)
		};
		gyro = new NavX(I2C.Port.kMXP);
		vision = Vision.getInstance();
		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		odometry = new SwerveDriveOdometry(kinematics, gyro.getAngleAsRotation2d(), getModulePositions(), vision.getRobotPose());
		poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getAngleAsRotation2d(), getModulePositions(), vision.getRobotPose());
	}

	public static synchronized SwerveDrive getInstance() {
		return instance == null ? new SwerveDrive() : instance;
	}

	@Override
	public void periodic() {
		odometry.update(gyro.getAngleAsRotation2d(), getModulePositions());
		poseEstimator.update(gyro.getAngleAsRotation2d(), getModulePositions());
		if (vision.getTagId() != -1) {
			poseEstimator.addVisionMeasurement(vision.getRobotPose(), Timer.getFPGATimestamp());
		}
	}

	public void driveFieldCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		driveModules(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity,
				new Rotation2d(getRobotAngle())
			)
		);
	}

	public void driveRobotCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		driveModules(
			new ChassisSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity
			)
		);
	}

	public void driveModules(ChassisSpeeds targetChassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			discretize(targetChassisSpeeds)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			SwerveConstants.MAX_LINEAR_SPEED
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	/** 
	 * Fixes situation where robot drifts in the direction it's rotating in if turning and translating at the same time 
	 * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
	*/
	private static ChassisSpeeds discretize(ChassisSpeeds originalChassisSpeeds) {
		double vx = originalChassisSpeeds.vxMetersPerSecond;
		double vy = originalChassisSpeeds.vyMetersPerSecond;
		double omega = originalChassisSpeeds.omegaRadiansPerSecond;
		double dt = 0.02; // This should be the time these values will be used, so normally just the loop time
		Pose2d desiredDeltaPose = new Pose2d(
			vx * dt,
			vy * dt,
			new Rotation2d(omega * dt)
		);
		Twist2d twist = new Pose2d().log(desiredDeltaPose);
		return new ChassisSpeeds(
			twist.dx / dt,
			twist.dy / dt,
			twist.dtheta / dt
		);
	}

	public NavX getGyro() {
		return gyro;
	}

	public void zeroModules() {
		for (SwerveModule module : modules) {
			module.setModuleVelocity(0);
			module.setModuleAngle(0);
		}
	}

	public Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getModuleState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getModulePosition();
		}
		return positions;
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public Pose2d getRobotPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Pose2d getRobotPoseNoVision() {
		return odometry.getPoseMeters();
	}

	public void resetPose(Pose2d newPose) {
		odometry.resetPosition(gyro.getAngleAsRotation2d(), getModulePositions(), newPose);
		poseEstimator.resetPosition(gyro.getAngleAsRotation2d(), getModulePositions(), newPose);
	}

	public double getRobotAngle() {
		return Math.toDegrees(gyro.getOffsetedAngle());
	}

	/**
	 * Sets the field-centric zero to some angle relative to the robot
	 * <p>CCW is positive
	 * @param offset the angle relative to the robot, in radians
	 */
	public void resetRobotAngle(double offset) {
		gyro.offsetGyroZero(offset);
		MessagingSystem.getInstance().addMessage("Swerve -> Reset Gyro");
	}

	public void resetRobotAngle() {
		resetRobotAngle(0);
	}

	public double getCurrentZero() {
		return gyro.getGyroZero();
	}

	@Override
	public void logData(Logger logger, LogTable table) {
		table.put("Front Left Module Velocity (M/S)", modules[0].getModuleState().speedMetersPerSecond);
		table.put("Front Left Module Angle (Radians)", modules[0].getModuleState().angle.getRadians());
		table.put("Front Right Module Velocity (M/S)", modules[1].getModuleState().speedMetersPerSecond);
		table.put("Front Right Module Angle (Radians)", modules[1].getModuleState().angle.getRadians());
		table.put("Back Left Module Velocity (M/S)", modules[2].getModuleState().speedMetersPerSecond);
		table.put("Back Left Module Angle (Radians)", modules[2].getModuleState().angle.getRadians());
		table.put("Back Right Module Velocity (M/S)", modules[3].getModuleState().speedMetersPerSecond);
		table.put("Back Right Module Angle (Radians)", modules[3].getModuleState().angle.getRadians());
		logger.recordOutput("Swerve Odometry", getRobotPoseNoVision());
		logger.recordOutput("Swerve + Vision Odometry", getRobotPose());
		logger.recordOutput("Module States", getModuleStates());
	}

	@Override
	public String getTableName() {
		return "Swerve";
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Gyro Angle: ", () -> gyro.getRawAngle(), null);
		builder.addDoubleProperty(
			"Gyro Offset From Zero: ",
			() -> getRobotAngle() % (2 * Math.PI),
			null
		);
		builder.addDoubleProperty(
			"Current Forward Speed: ",
			() -> getChassisSpeeds().vxMetersPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Current Sideways Speed: ",
			() -> getChassisSpeeds().vyMetersPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Current Rotational Speed: ",
			() -> getChassisSpeeds().omegaRadiansPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Estimated X: ",
			() -> getRobotPose().getX(),
			null
		);
		builder.addDoubleProperty(
			"Estimated Y: ",
			() -> getRobotPose().getY(),
			null
		);
		builder.addDoubleProperty(
			"Estimated Rotation: ",
			() -> getRobotPose().getRotation().getRadians(),
			null
		);
	}
}
