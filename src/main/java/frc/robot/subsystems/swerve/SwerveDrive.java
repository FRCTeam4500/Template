package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utilities.Loggable;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements Loggable {
	private static SwerveDrive instance;
	private NavX gyro;
	private Vision vision;
	private SwerveModule[] modules;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveDrivePoseEstimator poseEstimator;
	private PIDController anglePID;

	private SwerveDrive() {
		anglePID = new PIDController(4, 0, 0);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
		modules = new SwerveModule[] {
			new SwerveModule(
				SwerveConstants.FRONT_LEFT_DRIVE_MOTOR,
				SwerveConstants.FRONT_LEFT_ANGLE_MOTOR,
				SwerveConstants.FRONT_LEFT_MODULE_TRANSLATION
			),
			new SwerveModule(
				SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR,
				SwerveConstants.FRONT_RIGHT_ANGLE_MOTOR,
				SwerveConstants.FRONT_RIGHT_MODULE_TRANSLATION
			),
			new SwerveModule(
				SwerveConstants.BACK_LEFT_DRIVE_MOTOR,
				SwerveConstants.BACK_LEFT_ANGLE_MOTOR,
				SwerveConstants.BACK_LEFT_MODULE_TRANSLATION
			),
			new SwerveModule(
				SwerveConstants.BACK_RIGHT_DRIVE_MOTOR,
				SwerveConstants.BACK_RIGHT_ANGLE_MOTOR,
				SwerveConstants.BACK_RIGHT_MODULE_TRANSLATION
			),
		};
		gyro = new NavX(I2C.Port.kMXP);
		vision = Vision.getInstance();
		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		odometry = new SwerveDriveOdometry(
			kinematics,
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			vision.getRobotPose().orElse(new Pose2d())
		);
		poseEstimator = new SwerveDrivePoseEstimator(
			kinematics,
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			vision.getRobotPose().orElse(new Pose2d())
		);
	}

    public static synchronized SwerveDrive getInstance() {
		if (instance == null)
			instance = new SwerveDrive();
		return instance;
	}

	@Override
	public void periodic() {
		Rotation2d gyroAngle = gyro.getUnwrappedAngle();
		SwerveModulePosition[] modulePositions = getModulePositions();
		odometry.update(gyroAngle, modulePositions);
		poseEstimator.update(gyroAngle, modulePositions);
		if (vision.seesTag()) {
			poseEstimator.addVisionMeasurement(
                vision.getRobotPose().orElse(getEstimatorPose()),
                Timer.getFPGATimestamp()
            );
		}
	}

	public void driveAngleCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		Rotation2d targetRotation
	) {
		driveRobotCentric(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardVelocity,
                sidewaysVelocity,
                calculateRotationalVelocityToTarget(targetRotation),
                getRobotAngle()
            )
		);
	}

	public void driveAlignToTarget(
		double forwardVelocity,
		double leftVelocity,
		Rotation2d aligningAngle
	) {
		driveRobotCentric(
			new ChassisSpeeds(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					forwardVelocity,
					leftVelocity, 
					0, 
					getRobotAngle()
				).vxMetersPerSecond, 
				vision.getGamePieceHorizontalOffset()
                    .orElse(new Rotation2d())
                    .getDegrees() / 10, // Arbitrary scaling factor
				calculateRotationalVelocityToTarget(aligningAngle)
			)
		);
	}

	public void driveRobotCentric(ChassisSpeeds targetChassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			discretize(targetChassisSpeeds)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			SwerveConstants.MAX_LINEAR_SPEED_MPS
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public Pose2d getEstimatorPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Pose2d getOdometryPose() {
		return odometry.getPoseMeters();
	}

	public void resetPose(Pose2d newPose) {
		odometry.resetPosition(
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			newPose
		);
		poseEstimator.resetPosition(
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			newPose
		);
	}

	public Rotation2d getRobotAngle() {
		return gyro.getOffsetedAngle();
	}

	public void zeroRobotAngle() {
		gyro.zeroGyro();
	}

	public void resetRobotAngle(Rotation2d offsetAngle) {
		gyro.zeroGyroWithOffset(offsetAngle);
	}

	@Override
	public void logData(LogTable table) {
		table.put(
			"Front Left Module Velocity (M/S)",
			modules[0].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Front Left Module Angle (Radians)",
			modules[0].getModuleState().angle.getRadians()
		);
		table.put(
			"Front Right Module Velocity (M/S)",
			modules[1].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Front Right Module Angle (Radians)",
			modules[1].getModuleState().angle.getRadians()
		);
		table.put(
			"Back Left Module Velocity (M/S)",
			modules[2].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Back Left Module Angle (Radians)",
			modules[2].getModuleState().angle.getRadians()
		);
		table.put(
			"Back Right Module Velocity (M/S)",
			modules[3].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Back Right Module Angle (Radians)",
			modules[3].getModuleState().angle.getRadians()
		);
		Logger.getInstance().recordOutput("Swerve Odometry", getOdometryPose());
		Logger.getInstance().recordOutput("Odometyry + Vision Pose Estimation", getEstimatorPose());
		Logger.getInstance().recordOutput("Module States", getModuleStates());
	}

	@Override
	public String getTableName() {
		return "Swerve";
	}

	private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		if (anglePID.atSetpoint()) {
			return 0;
		} else {
			return anglePID.calculate(getRobotAngle().getRadians(), targetRotation.getRadians());
		}
	}

	/**
	 * Fixes situation where robot drifts in the direction it's rotating in if
	 * turning and translating at the same time
	 * 
	 * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
	 */
	private ChassisSpeeds discretize(
		ChassisSpeeds originalChassisSpeeds
    ) {
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

	private Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getModuleState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getModulePosition();
		}
		return positions;
	}
}