package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.Loggable;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase implements Loggable {
	private static Vision instance;
	private Limelight aprilTagLimelight;
	private Limelight gamePieceLimelight;

	//TODO: Change these
	private double GAMEPIECE_LIMELIGHT_HEIGHT_METERS = 0;
	private double GAMEPIECE_HALF_HEIGHT_METERS = 0;
	private Rotation2d GAMEPIECE_LIMELIGHT_ANGLE = Rotation2d.fromDegrees(0);

	private Vision() {
		aprilTagLimelight = new Limelight("limelight-hehehe");
		gamePieceLimelight = new Limelight("limelight-haha");
	}

	public static synchronized Vision getInstance() {
		if (instance == null) instance = new Vision();
		return instance;
	}

	@Override
	public void logData(LogTable table) {
		table.put("Tag ID", getTagId(0));
        table.put("Sees tag", seesTag());
        table.put("Sees gamepiece", seesGamePiece());
		Logger.getInstance().recordOutput("Vision Odometry", getRobotPose(new Pose2d()));
	}

	@Override
	public String getTableName() {
		return "Vision";
	}

	public Limelight getAprilTageLimelight() {
		return aprilTagLimelight;
	}

	public Limelight getGamePieceLimelight() {
		return gamePieceLimelight;
	}

	public boolean seesTag() {
		return aprilTagLimelight.hasValidTargets();
	}

	public boolean seesGamePiece() {
		return gamePieceLimelight.hasValidTargets();
	}

	public ChassisSpeeds alignToGamePiece(
		Rotation2d horizontalOffset,
		Rotation2d verticalOffset
	) {
		return new ChassisSpeeds(
			getHorizontalOffset(new Rotation2d()).minus(horizontalOffset).getDegrees() / 10, 
			getHorizontalOffset(new Rotation2d()).minus(horizontalOffset).getDegrees() / 10, 
			0
		);
	}

	public Translation2d getTranslation(Translation2d defaultTranslation) {
		if (!seesGamePiece()) return defaultTranslation;
		double forwardDistance = 
			(GAMEPIECE_LIMELIGHT_HEIGHT_METERS - GAMEPIECE_HALF_HEIGHT_METERS) / 
			Math.tan(
				GAMEPIECE_LIMELIGHT_ANGLE.plus(
					getVerticalOffset(new Rotation2d())
				).getRadians()
			);
		return new Translation2d(
			forwardDistance,
			forwardDistance * Math.tan(
				getVerticalOffset(new Rotation2d())
				.getRadians()
			)
		);
	}

	public int getTagId(int defaultID) {
		return aprilTagLimelight.getTargetTagId().orElse(defaultID);
	}

	public Pose2d getRobotPose(Pose2d defaultPose) {
		return getRobotPose(defaultPose, DriverStation.getAlliance());
	}

	public Pose2d getRobotPose(Pose2d defaultPose, Alliance poseOrigin) {
		return aprilTagLimelight
			.getRobotPoseToAlliance(poseOrigin)
			.orElse(defaultPose);
	}

	public Pose2d getRelativeAprilTagPose(Pose2d defaultPose) {
		return aprilTagLimelight
			.getTargetPoseToRobot()
			.orElse(defaultPose);
	}

	public Rotation2d getHorizontalOffset(Rotation2d defaultRotation) {
		return gamePieceLimelight
			.getHorizontalOffsetFromCrosshair()
			.orElse(defaultRotation);
	}

	public Rotation2d getVerticalOffset(Rotation2d defaultRotation) {
		return gamePieceLimelight
			.getVerticalOffsetFromCrosshair()
			.orElse(defaultRotation);
	}

	public double getTakenArea(double defaultArea) {
		return gamePieceLimelight
			.getTargetArea()
			.orElse(defaultArea);
	}

	public Rotation2d getSkew(Rotation2d defaultSkew) {
		return gamePieceLimelight
			.getSkew()
			.orElse(defaultSkew);
	}
}
